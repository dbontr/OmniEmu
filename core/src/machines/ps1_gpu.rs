use std::collections::VecDeque;

use crate::kernel::VideoBuffer;
use crate::state::{StateReader, StateWriter};

const VRAM_WIDTH: usize = 1024;
const VRAM_HEIGHT: usize = 512;
const OUTPUT_WIDTH: u32 = 320;
const OUTPUT_HEIGHT: u32 = 240;
const GP0_FIFO_WORDS: usize = 16;

#[derive(Clone, Copy, Default)]
struct Transfer {
    x: u16,
    y: u16,
    width: u16,
    height: u16,
    index: u32,
    total: u32,
}

#[derive(Clone, Copy, Default)]
struct RenderVertex {
    x: i32,
    y: i32,
    color: [i32; 3],
    u: i32,
    v: i32,
}

#[derive(Clone, Copy)]
struct AttributePlane {
    base: u32,
    step_x: u32,
    step_y: u32,
}

impl AttributePlane {
    const ATTR_SHIFT: u32 = 12;
    const POST_SHIFT: u32 = 12;

    fn add_steps(value: u32, step: u32, count: i32) -> u32 {
        value.wrapping_add((step as i32).wrapping_mul(count) as u32)
    }

    fn new(vertices: [RenderVertex; 3], values: [i32; 3], anchor: usize, determinant: i64) -> Self {
        let dx_num = i64::from(values[1] - values[0]) * i64::from(vertices[2].y - vertices[1].y)
            - i64::from(values[2] - values[1]) * i64::from(vertices[1].y - vertices[0].y);
        let dy_num = i64::from(vertices[1].x - vertices[0].x) * i64::from(values[2] - values[1])
            - i64::from(vertices[2].x - vertices[1].x) * i64::from(values[1] - values[0]);
        let step_x_12 = ((dx_num << Self::ATTR_SHIFT) / determinant) as i32;
        let step_y_12 = ((dy_num << Self::ATTR_SHIFT) / determinant) as i32;
        let step_x = (step_x_12 as u32) << Self::POST_SHIFT;
        let step_y = (step_y_12 as u32) << Self::POST_SHIFT;
        let initial = ((values[anchor] as u32) << (Self::ATTR_SHIFT + Self::POST_SHIFT))
            .wrapping_add(1 << (Self::ATTR_SHIFT + Self::POST_SHIFT - 1));
        let base = Self::add_steps(
            Self::add_steps(initial, step_x, -vertices[anchor].x),
            step_y,
            -vertices[anchor].y,
        );
        Self {
            base,
            step_x,
            step_y,
        }
    }

    fn sample(self, x: i32, y: i32) -> i32 {
        let value = Self::add_steps(Self::add_steps(self.base, self.step_x, x), self.step_y, y);
        (value >> (Self::ATTR_SHIFT + Self::POST_SHIFT)) as i32
    }
}

pub struct Ps1Gpu {
    vram: Vec<u16>,
    video: VideoBuffer,
    command: Vec<u32>,
    fifo: VecDeque<u32>,
    transfer_write: Option<Transfer>,
    transfer_read: Option<Transfer>,
    display_x: u16,
    display_y: u16,
    display_h_start: u16,
    display_h_end: u16,
    display_v_start: u16,
    display_v_end: u16,
    display_mode: u8,
    display_disabled: bool,
    dma_direction: u8,
    draw_mode: u16,
    texture_window: u32,
    clut_cache: [u16; 256],
    clut_cache_key: u16,
    clut_cache_len: u16,
    draw_area_min: (u16, u16),
    draw_area_max: (u16, u16),
    draw_offset: (i16, i16),
    force_mask: bool,
    check_mask: bool,
    read_latch: u32,
    status: u32,
    frame: u64,
    cycle_phase: u64,
    pending_command_ticks: u64,
    irq: bool,
}
impl Default for Ps1Gpu {
    fn default() -> Self {
        Self::new()
    }
}

impl Ps1Gpu {
    pub fn new() -> Self {
        let mut video = VideoBuffer::new(OUTPUT_WIDTH, OUTPUT_HEIGHT);
        video.clear([0, 0, 0, 255]);
        Self {
            vram: vec![0; VRAM_WIDTH * VRAM_HEIGHT],
            video,
            command: Vec::with_capacity(16),
            fifo: VecDeque::with_capacity(GP0_FIFO_WORDS),
            transfer_write: None,
            transfer_read: None,
            display_x: 0,
            display_y: 0,
            display_h_start: 0x0200,
            display_h_end: 0x0c00,
            display_v_start: 0x0010,
            display_v_end: 0x0100,
            display_mode: 0,
            display_disabled: true,
            dma_direction: 0,
            draw_mode: 0,
            texture_window: 0,
            clut_cache: [0; 256],
            clut_cache_key: 0,
            clut_cache_len: 0,
            draw_area_min: (0, 0),
            draw_area_max: (1023, 511),
            draw_offset: (0, 0),
            force_mask: false,
            check_mask: false,
            read_latch: 0,
            status: 0x1480_2000,
            frame: 0,
            cycle_phase: 0,
            pending_command_ticks: 0,
            irq: false,
        }
    }

    pub fn reset(&mut self) {
        let vram = std::mem::take(&mut self.vram);
        *self = Self::new();
        self.vram = vram;
        self.vram.fill(0);
    }
    pub fn video(&self) -> &VideoBuffer {
        &self.video
    }
    pub fn frame(&self) -> u64 {
        self.frame
    }
    pub fn irq_pending(&self) -> bool {
        self.irq
    }

    fn command_words(opcode: u8) -> usize {
        match opcode {
            0x00 | 0x01 | 0x1f | 0xe1..=0xe6 => 1,
            0x02 => 3,
            0x20..=0x3f => {
                let vertices = if opcode & 0x08 != 0 { 4 } else { 3 };
                let textured = opcode & 0x04 != 0;
                let gouraud = opcode & 0x10 != 0;
                1 + vertices
                    + usize::from(textured) * vertices
                    + usize::from(gouraud) * (vertices - 1)
            }
            0x40..=0x5f => {
                if opcode & 0x10 != 0 {
                    4
                } else {
                    3
                }
            }
            0x60..=0x7f => {
                let variable_size = opcode & 0x18 == 0;
                2 + usize::from(opcode & 0x04 != 0) + usize::from(variable_size)
            }
            0x80..=0x9f => 4,
            0xa0..=0xdf => 3,
            _ => 1,
        }
    }

    fn clipped_timing_point(&self, point: (i32, i32)) -> (i32, i32) {
        (
            point.0.clamp(
                i32::from(self.draw_area_min.0),
                i32::from(self.draw_area_max.0),
            ),
            point.1.clamp(
                i32::from(self.draw_area_min.1),
                i32::from(self.draw_area_max.1),
            ),
        )
    }

    fn polygon_timing_vertices(&self, command: &[u32], opcode: u8) -> [(i32, i32); 4] {
        let gouraud = opcode & 0x10 != 0;
        let textured = opcode & 0x04 != 0;
        let vertex_count = if opcode & 0x08 != 0 { 4 } else { 3 };
        let mut points = [(0, 0); 4];
        let mut packet_index = 1usize;
        for (vertex_index, point) in points.iter_mut().enumerate().take(vertex_count) {
            if gouraud && vertex_index != 0 {
                packet_index += 1;
            }
            *point = self.vertex(command[packet_index]);
            packet_index += 1;
            if textured {
                packet_index += 1;
            }
        }
        points
    }

    fn triangle_timing_ticks(
        &self,
        vertices: [(i32, i32); 3],
        textured: bool,
        semi_transparent: bool,
    ) -> u64 {
        let vertices = vertices.map(|point| self.clipped_timing_point(point));
        let mut pixels = Self::edge(vertices[0], vertices[1], vertices[2]).unsigned_abs() / 2;
        if textured {
            pixels = pixels.saturating_mul(2);
        }
        if semi_transparent || self.check_mask {
            pixels = pixels.saturating_add(pixels.div_ceil(2));
        }
        if self.interleaved_480i_mode() && self.draw_mode & (1 << 10) == 0 {
            pixels /= 2;
        }
        pixels
    }

    fn polygon_command_ticks(&self, command: &[u32], opcode: u8) -> u64 {
        const SETUP: [[[u64; 2]; 2]; 2] = [[[46, 226], [334, 496]], [[82, 262], [370, 532]]];
        let quad = opcode & 0x08 != 0;
        let gouraud = opcode & 0x10 != 0;
        let textured = opcode & 0x04 != 0;
        let semi_transparent = opcode & 0x02 != 0;
        let points = self.polygon_timing_vertices(command, opcode);
        let mut ticks = SETUP[usize::from(quad)][usize::from(gouraud)][usize::from(textured)];
        ticks = ticks.saturating_add(self.triangle_timing_ticks(
            [points[0], points[1], points[2]],
            textured,
            semi_transparent,
        ));
        if quad {
            ticks = ticks.saturating_add(self.triangle_timing_ticks(
                [points[2], points[1], points[3]],
                textured,
                semi_transparent,
            ));
        }
        ticks
    }

    fn clipped_timing_rect(&self, x: i32, y: i32, width: i32, height: i32) -> (u64, u64) {
        let left = x.max(i32::from(self.draw_area_min.0)).max(0);
        let top = y.max(i32::from(self.draw_area_min.1)).max(0);
        let right = (x + width)
            .min(i32::from(self.draw_area_max.0) + 1)
            .min(VRAM_WIDTH as i32);
        let bottom = (y + height)
            .min(i32::from(self.draw_area_max.1) + 1)
            .min(VRAM_HEIGHT as i32);
        (
            u64::try_from((right - left).max(0)).unwrap_or(0),
            u64::try_from((bottom - top).max(0)).unwrap_or(0),
        )
    }

    fn rectangle_command_ticks(&self, command: &[u32], opcode: u8) -> u64 {
        let textured = opcode & 0x04 != 0;
        let semi_transparent = opcode & 0x02 != 0;
        let size_code = (opcode >> 3) & 3;
        let (x, y) = self.vertex(command[1]);
        let mut packet_index = 2usize;
        if textured {
            packet_index += 1;
        }
        let (width, height) = match size_code {
            0 => {
                let (width, height) = Self::wh(command[packet_index]);
                (i32::from(width & 0x03ff), i32::from(height & 0x01ff))
            }
            1 => (1, 1),
            2 => (8, 8),
            _ => (16, 16),
        };
        let (width, mut height) = self.clipped_timing_rect(x, y, width, height);
        if width == 0 || height == 0 {
            return 16;
        }
        let mut row_ticks = width;
        if textured {
            match (self.draw_mode >> 7) & 3 {
                0 => row_ticks = row_ticks.saturating_add(width),
                1 if width > 128 => row_ticks = row_ticks.saturating_add((width / 4) * 8),
                1 if width * height > 2048 => {
                    row_ticks = row_ticks.saturating_add((width / 4) * (4 * (128 / width).max(1)))
                }
                1 => row_ticks = row_ticks.saturating_add(width),
                _ if width > 128 => row_ticks = row_ticks.saturating_add((width / 2) * 8),
                _ if width * height > 1024 => {
                    row_ticks = row_ticks.saturating_add((width / 4) * (8 * (128 / width).max(1)))
                }
                _ => row_ticks = row_ticks.saturating_add(width),
            }
        }
        if semi_transparent || self.check_mask {
            row_ticks = row_ticks.saturating_add(width.div_ceil(2));
        }
        if self.interleaved_480i_mode() && self.draw_mode & (1 << 10) == 0 {
            height = (height / 2).max(1);
        }
        16u64.saturating_add(row_ticks.saturating_mul(height))
    }

    fn line_command_ticks(&self, command: &[u32], opcode: u8) -> u64 {
        let gouraud = opcode & 0x10 != 0;
        let polyline = opcode & 0x08 != 0;
        let mut packet_index = 1usize;
        let mut points = Vec::new();
        if packet_index >= command.len() {
            return 1;
        }
        points.push(self.vertex(command[packet_index]));
        packet_index += 1;
        while packet_index < command.len() {
            if gouraud {
                if command[packet_index] & 0xf000_f000 == 0x5000_5000 {
                    break;
                }
                packet_index += 1;
                if packet_index >= command.len() {
                    break;
                }
            } else if polyline && command[packet_index] & 0xf000_f000 == 0x5000_5000 {
                break;
            }
            points.push(self.vertex(command[packet_index]));
            packet_index += 1;
            if !polyline && points.len() == 2 {
                break;
            }
        }
        points
            .windows(2)
            .map(|segment| {
                let a = self.clipped_timing_point(segment[0]);
                let b = self.clipped_timing_point(segment[1]);
                let width = i64::from((b.0 - a.0).abs()) + 1;
                let mut height = i64::from((b.1 - a.1).abs()) + 1;
                if self.interleaved_480i_mode() && self.draw_mode & (1 << 10) == 0 {
                    height = (height / 2).max(1);
                }
                u64::try_from(width.max(height)).unwrap_or(1)
            })
            .sum::<u64>()
            .max(1)
    }

    fn command_execution_ticks(&self, command: &[u32]) -> u64 {
        let opcode = (command[0] >> 24) as u8;
        match opcode {
            0x00 | 0x01 | 0x1f | 0xe1..=0xe6 => 1,
            0x02 => {
                let raw_width = u64::from((command[2] as u16) & 0x03ff);
                let width = (raw_width + 0x0f) & !0x0f;
                let height = u64::from(((command[2] >> 16) as u16) & 0x01ff);
                46u64.saturating_add((width / 8 + 9).saturating_mul(height))
            }
            0x20..=0x3f => self.polygon_command_ticks(command, opcode),
            0x40..=0x5f => self.line_command_ticks(command, opcode),
            0x60..=0x7f => self.rectangle_command_ticks(command, opcode),
            0x80..=0x9f => {
                let (width, height) = Self::wh(command[3]);
                u64::from(Self::normalized_extent(width, 0x03ff))
                    .saturating_mul(u64::from(Self::normalized_extent(height, 0x01ff)))
                    .saturating_mul(2)
            }
            0xa0..=0xdf => 0,
            _ => 1,
        }
    }

    fn fifo_occupancy(&self) -> usize {
        self.fifo.len().saturating_add(self.command.len())
    }

    pub fn can_accept_gp0_word(&self) -> bool {
        self.transfer_read.is_none() && self.fifo_occupancy() < GP0_FIFO_WORDS
    }

    pub fn can_read_vram_word(&self) -> bool {
        self.transfer_read.is_some()
    }

    pub fn dma_request(&mut self) -> bool {
        self.status() & (1 << 25) != 0
    }

    pub fn gp0(&mut self, value: u32) -> bool {
        let opcode = (value >> 24) as u8;
        if self.command.is_empty()
            && self.transfer_write.is_none()
            && self.transfer_read.is_none()
            && (0xe3..=0xe5).contains(&opcode)
        {
            self.execute_gp0(&[value]);
            return true;
        }
        if !self.can_accept_gp0_word() {
            self.refresh_status();
            return false;
        }
        self.fifo.push_back(value);
        self.process_gp0_fifo();
        self.refresh_status();
        true
    }

    fn process_gp0_fifo(&mut self) {
        loop {
            if self.pending_command_ticks != 0 || self.transfer_read.is_some() {
                break;
            }
            if let Some(mut transfer) = self.transfer_write.take() {
                let Some(value) = self.fifo.pop_front() else {
                    self.transfer_write = Some(transfer);
                    break;
                };
                self.write_transfer_half(&mut transfer, value as u16);
                if transfer.index < transfer.total {
                    self.write_transfer_half(&mut transfer, (value >> 16) as u16);
                }
                if transfer.index < transfer.total {
                    self.transfer_write = Some(transfer);
                }
                continue;
            }
            if self.command.is_empty() {
                let Some(value) = self.fifo.pop_front() else {
                    break;
                };
                self.command.push(value);
            }

            let opcode = (self.command[0] >> 24) as u8;
            if (0x40..=0x5f).contains(&opcode) && opcode & 0x08 != 0 {
                let gouraud = opcode & 0x10 != 0;
                let minimum_words = if gouraud { 5 } else { 4 };
                let terminated = self.command.len() >= minimum_words
                    && self
                        .command
                        .last()
                        .is_some_and(|word| word & 0xf000_f000 == 0x5000_5000);
                if terminated {
                    let command = std::mem::take(&mut self.command);
                    self.execute_gp0(&command);
                    continue;
                }
            } else if self.command.len() >= Self::command_words(opcode) {
                let command = std::mem::take(&mut self.command);
                self.execute_gp0(&command);
                continue;
            }

            let Some(value) = self.fifo.pop_front() else {
                break;
            };
            self.command.push(value);
        }
    }
    fn execute_gp0(&mut self, command: &[u32]) {
        let opcode = (command[0] >> 24) as u8;
        let command_ticks = self.command_execution_ticks(command);
        match opcode {
            0x00 => {}
            0x01 => self.clut_cache_len = 0,
            0x1f => self.request_irq(),
            0x02 => self.fill_rect(command),
            0x20..=0x3f => self.polygon(command),
            0x40..=0x5f => self.line(command),
            0x60..=0x7f => self.rectangle(command),
            0x80..=0x9f => self.copy_vram(command),
            0xa0..=0xbf => self.begin_cpu_to_vram(command),
            0xc0..=0xdf => self.begin_vram_to_cpu(command),
            0xe1 => self.set_draw_mode(command[0]),
            0xe2 => self.texture_window = command[0] & 0x000f_ffff,
            0xe3 => {
                self.draw_area_min = (
                    (command[0] & 0x03ff) as u16,
                    ((command[0] >> 10) & 0x01ff) as u16,
                )
            }
            0xe4 => {
                self.draw_area_max = (
                    (command[0] & 0x03ff) as u16,
                    ((command[0] >> 10) & 0x01ff) as u16,
                )
            }
            0xe5 => self.set_draw_offset(command[0]),
            0xe6 => {
                self.force_mask = command[0] & 1 != 0;
                self.check_mask = command[0] & 2 != 0;
            }
            _ => {}
        }
        self.pending_command_ticks = self.pending_command_ticks.saturating_add(command_ticks);
        self.refresh_status();
    }

    fn color15(value: u32) -> u16 {
        let r = (value & 0xff) >> 3;
        let g = ((value >> 8) & 0xff) >> 3;
        let b = ((value >> 16) & 0xff) >> 3;
        (r | (g << 5) | (b << 10)) as u16
    }

    fn color8(value: u32) -> [i32; 3] {
        [
            (value & 0xff) as i32,
            ((value >> 8) & 0xff) as i32,
            ((value >> 16) & 0xff) as i32,
        ]
    }

    fn sign_extend_11(value: u32) -> i16 {
        (((value & 0x07ff) as i16) << 5) >> 5
    }

    fn xy(value: u32) -> (i32, i32) {
        (
            i32::from(Self::sign_extend_11(value)),
            i32::from(Self::sign_extend_11(value >> 16)),
        )
    }

    fn vertex(&self, value: u32) -> (i32, i32) {
        let (x, y) = Self::xy(value);
        (
            x + i32::from(self.draw_offset.0),
            y + i32::from(self.draw_offset.1),
        )
    }

    fn set_draw_mode(&mut self, value: u32) {
        self.draw_mode = (value & 0x3fff) as u16;
    }

    fn set_draw_offset(&mut self, value: u32) {
        self.draw_offset = (
            Self::sign_extend_11(value),
            Self::sign_extend_11(value >> 11),
        );
    }
    fn wh(value: u32) -> (u16, u16) {
        ((value & 0xffff) as u16, ((value >> 16) & 0xffff) as u16)
    }

    fn vram_index(x: i32, y: i32) -> usize {
        let x = x.rem_euclid(VRAM_WIDTH as i32) as usize;
        let y = y.rem_euclid(VRAM_HEIGHT as i32) as usize;
        y * VRAM_WIDTH + x
    }

    fn write_pixel(&mut self, x: i32, y: i32, color: u16) {
        let index = Self::vram_index(x, y);
        self.vram[index] = color;
    }

    fn write_transfer_pixel(&mut self, x: i32, y: i32, color: u16) {
        let index = Self::vram_index(x, y);
        if self.check_mask && self.vram[index] & 0x8000 != 0 {
            return;
        }
        self.vram[index] = (color & 0x7fff)
            | if self.force_mask {
                0x8000
            } else {
                color & 0x8000
            };
    }

    fn in_draw_area(&self, x: i32, y: i32) -> bool {
        x >= i32::from(self.draw_area_min.0)
            && x <= i32::from(self.draw_area_max.0)
            && y >= i32::from(self.draw_area_min.1)
            && y <= i32::from(self.draw_area_max.1)
            && (0..VRAM_WIDTH as i32).contains(&x)
            && (0..VRAM_HEIGHT as i32).contains(&y)
    }

    fn interleaved_480i_mode(&self) -> bool {
        self.display_mode & 0x24 == 0x24
    }

    fn interlaced_display_field(&self) -> u8 {
        (self.frame & 1) as u8
    }

    fn active_line_lsb(&self) -> u8 {
        ((u32::from(self.display_y) + u32::from(self.interlaced_display_field())) & 1) as u8
    }

    fn skip_render_line(&self, y: i32) -> bool {
        self.interleaved_480i_mode()
            && self.draw_mode & (1 << 10) == 0
            && ((y as u32) & 1) as u8 == self.active_line_lsb()
    }

    fn blend_pixel(back: u16, front: u16, mode: u16) -> u16 {
        let blend = |shift: u16| -> u16 {
            let b = i32::from((back >> shift) & 0x1f);
            let f = i32::from((front >> shift) & 0x1f);
            let value = match mode & 3 {
                0 => b / 2 + f / 2,
                1 => b + f,
                2 => b - f,
                _ => b + f / 4,
            };
            value.clamp(0, 31) as u16
        };
        blend(0) | (blend(5) << 5) | (blend(10) << 10)
    }

    fn write_render_pixel(&mut self, x: i32, y: i32, color: u16, semi_transparent: bool) {
        if !self.in_draw_area(x, y) || self.skip_render_line(y) {
            return;
        }
        let index = Self::vram_index(x, y);
        if self.check_mask && self.vram[index] & 0x8000 != 0 {
            return;
        }
        let rgb = if semi_transparent {
            Self::blend_pixel(self.vram[index], color, self.draw_mode >> 5)
        } else {
            color & 0x7fff
        };
        self.vram[index] = rgb
            | if self.force_mask {
                0x8000
            } else {
                color & 0x8000
            };
    }

    fn texture_uv(&self, u: i32, v: i32) -> (u8, u8) {
        let mask_x = (self.texture_window & 0x1f) as u8;
        let mask_y = ((self.texture_window >> 5) & 0x1f) as u8;
        let offset_x = ((self.texture_window >> 10) & 0x1f) as u8;
        let offset_y = ((self.texture_window >> 15) & 0x1f) as u8;
        let apply = |coord: i32, mask: u8, offset: u8| -> u8 {
            let coord = coord as u8;
            (coord & !(mask << 3)) | ((offset & mask) << 3)
        };
        (apply(u, mask_x, offset_x), apply(v, mask_y, offset_y))
    }

    fn prepare_clut_cache(&mut self, tpage: u16, clut: u16) {
        let required = match (tpage >> 7) & 3 {
            0 => 16usize,
            1 => 256usize,
            _ => return,
        };
        let key = clut & 0x7fff;
        if self.clut_cache_key == key && usize::from(self.clut_cache_len) >= required {
            return;
        }
        let base_x = i32::from(clut & 0x3f) * 16;
        let base_y = i32::from((clut >> 6) & 0x01ff);
        for index in 0..required {
            self.clut_cache[index] = self.vram[Self::vram_index(base_x + index as i32, base_y)];
        }
        self.clut_cache_key = key;
        self.clut_cache_len = required as u16;
    }

    fn cached_clut_pixel(&self, clut: u16, palette: usize) -> u16 {
        let key = clut & 0x7fff;
        if self.clut_cache_key == key && palette < usize::from(self.clut_cache_len) {
            return self.clut_cache[palette];
        }
        let clut_x = i32::from(clut & 0x3f) * 16 + palette as i32;
        let clut_y = i32::from((clut >> 6) & 0x01ff);
        self.vram[Self::vram_index(clut_x, clut_y)]
    }

    fn texture_pixel(&self, tpage: u16, clut: u16, u: i32, v: i32) -> u16 {
        let (u, v) = self.texture_uv(u, v);
        let base_x = i32::from(tpage & 0x0f) * 64;
        let base_y = i32::from((tpage >> 4) & 1) * 256;
        match (tpage >> 7) & 3 {
            0 => {
                let word =
                    self.vram[Self::vram_index(base_x + i32::from(u / 4), base_y + i32::from(v))];
                let palette = usize::from((word >> ((u & 3) * 4)) & 0x0f);
                self.cached_clut_pixel(clut, palette)
            }
            1 => {
                let word =
                    self.vram[Self::vram_index(base_x + i32::from(u / 2), base_y + i32::from(v))];
                let palette = usize::from(if u & 1 == 0 { word & 0xff } else { word >> 8 });
                self.cached_clut_pixel(clut, palette)
            }
            _ => self.vram[Self::vram_index(base_x + i32::from(u), base_y + i32::from(v))],
        }
    }

    fn dither_value(x: i32, y: i32) -> i32 {
        const DITHER: [[i32; 4]; 4] = [
            [-4, 0, -3, 1],
            [2, -2, 3, -1],
            [-3, 1, -4, 0],
            [3, -1, 2, -2],
        ];
        DITHER[(y & 3) as usize][(x & 3) as usize]
    }

    fn rgb8_to_15(rgb: [i32; 3], x: i32, y: i32, dither: bool) -> u16 {
        let adjustment = if dither { Self::dither_value(x, y) } else { 0 };
        let channel = |value: i32| -> u16 { ((value + adjustment).clamp(0, 255) >> 3) as u16 };
        channel(rgb[0]) | (channel(rgb[1]) << 5) | (channel(rgb[2]) << 10)
    }

    fn modulate_texture(texture: u16, rgb: [i32; 3], x: i32, y: i32, dither: bool) -> u16 {
        let channel = |shift: u16, color: i32| -> i32 {
            let texel = i32::from((texture >> shift) & 0x1f);
            ((texel * color) >> 4).clamp(0, 255)
        };
        Self::rgb8_to_15(
            [channel(0, rgb[0]), channel(5, rgb[1]), channel(10, rgb[2])],
            x,
            y,
            dither,
        ) | (texture & 0x8000)
    }

    fn fill_rect(&mut self, command: &[u32]) {
        let color = Self::color15(command[0]);
        let x = i32::from((command[1] as u16) & 0x03f0);
        let y = i32::from(((command[1] >> 16) as u16) & 0x01ff);
        let raw_width = u32::from((command[2] as u16) & 0x03ff);
        let width = ((raw_width + 0x0f) & !0x0f) as i32;
        let height = i32::from(((command[2] >> 16) as u16) & 0x01ff);
        for dy in 0..height {
            let line_y = y + dy;
            if self.skip_render_line(line_y) {
                continue;
            }
            for dx in 0..width {
                self.write_pixel(x + dx, line_y, color);
            }
        }
    }

    fn edge(a: (i32, i32), b: (i32, i32), p: (i32, i32)) -> i64 {
        i64::from(p.0 - a.0) * i64::from(b.1 - a.1) - i64::from(p.1 - a.1) * i64::from(b.0 - a.0)
    }

    fn polygon_edge_inclusive(a: (i32, i32), b: (i32, i32), area: i64) -> bool {
        let dx = b.0 - a.0;
        let dy = b.1 - a.1;
        if area > 0 {
            dy > 0 || (dy == 0 && dx < 0)
        } else {
            dy < 0 || (dy == 0 && dx > 0)
        }
    }

    fn render_triangle(&mut self, vertices: [RenderVertex; 3], opcode: u8, tpage: u16, clut: u16) {
        let gouraud = opcode & 0x10 != 0;
        let textured = opcode & 0x04 != 0;
        let semi_transparent = opcode & 0x02 != 0;
        let raw_texture = opcode & 0x01 != 0;
        let points = [
            (vertices[0].x, vertices[0].y),
            (vertices[1].x, vertices[1].y),
            (vertices[2].x, vertices[2].y),
        ];
        let min_vertex_x = points.iter().map(|point| point.0).min().unwrap_or(0);
        let max_vertex_x = points.iter().map(|point| point.0).max().unwrap_or(0);
        let min_vertex_y = points.iter().map(|point| point.1).min().unwrap_or(0);
        let max_vertex_y = points.iter().map(|point| point.1).max().unwrap_or(0);
        if max_vertex_x - min_vertex_x > 1023 || max_vertex_y - min_vertex_y > 511 {
            return;
        }
        let min_x = min_vertex_x.max(i32::from(self.draw_area_min.0)).max(0);
        let max_x = max_vertex_x
            .min(i32::from(self.draw_area_max.0))
            .min((VRAM_WIDTH - 1) as i32);
        let min_y = min_vertex_y.max(i32::from(self.draw_area_min.1)).max(0);
        let max_y = max_vertex_y
            .min(i32::from(self.draw_area_max.1))
            .min((VRAM_HEIGHT - 1) as i32);
        if min_x > max_x || min_y > max_y {
            return;
        }
        let area = Self::edge(points[0], points[1], points[2]);
        if area == 0 {
            return;
        }
        let determinant = -area;
        let anchor = if vertices[1].x <= vertices[0].x {
            if vertices[2].x <= vertices[1].x {
                2
            } else {
                1
            }
        } else if vertices[2].x < vertices[0].x {
            2
        } else {
            0
        };
        let color_planes = if gouraud {
            Some([
                AttributePlane::new(
                    vertices,
                    [
                        vertices[0].color[0],
                        vertices[1].color[0],
                        vertices[2].color[0],
                    ],
                    anchor,
                    determinant,
                ),
                AttributePlane::new(
                    vertices,
                    [
                        vertices[0].color[1],
                        vertices[1].color[1],
                        vertices[2].color[1],
                    ],
                    anchor,
                    determinant,
                ),
                AttributePlane::new(
                    vertices,
                    [
                        vertices[0].color[2],
                        vertices[1].color[2],
                        vertices[2].color[2],
                    ],
                    anchor,
                    determinant,
                ),
            ])
        } else {
            None
        };
        let texture_planes = if textured {
            Some((
                AttributePlane::new(
                    vertices,
                    [vertices[0].u, vertices[1].u, vertices[2].u],
                    anchor,
                    determinant,
                ),
                AttributePlane::new(
                    vertices,
                    [vertices[0].v, vertices[1].v, vertices[2].v],
                    anchor,
                    determinant,
                ),
            ))
        } else {
            None
        };
        if textured {
            self.prepare_clut_cache(tpage, clut);
        }
        let dither = self.draw_mode & (1 << 9) != 0;
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let point = (x, y);
                let weights = [
                    Self::edge(points[1], points[2], point),
                    Self::edge(points[2], points[0], point),
                    Self::edge(points[0], points[1], point),
                ];
                let edges = [
                    (points[1], points[2]),
                    (points[2], points[0]),
                    (points[0], points[1]),
                ];
                let inside = weights.iter().zip(edges).all(|(weight, (edge_a, edge_b))| {
                    if area > 0 {
                        *weight > 0
                            || (*weight == 0 && Self::polygon_edge_inclusive(edge_a, edge_b, area))
                    } else {
                        *weight < 0
                            || (*weight == 0 && Self::polygon_edge_inclusive(edge_a, edge_b, area))
                    }
                });
                if !inside {
                    continue;
                }
                let rgb = if let Some(planes) = color_planes {
                    [
                        planes[0].sample(x, y),
                        planes[1].sample(x, y),
                        planes[2].sample(x, y),
                    ]
                } else {
                    vertices[0].color
                };
                if let Some((u_plane, v_plane)) = texture_planes {
                    let u = u_plane.sample(x, y);
                    let v = v_plane.sample(x, y);
                    let texture = self.texture_pixel(tpage, clut, u, v);
                    if texture == 0 {
                        continue;
                    }
                    let color = if raw_texture {
                        texture
                    } else {
                        Self::modulate_texture(texture, rgb, x, y, dither)
                    };
                    self.write_render_pixel(x, y, color, semi_transparent && texture & 0x8000 != 0);
                } else {
                    let color = Self::rgb8_to_15(rgb, x, y, gouraud && dither);
                    self.write_render_pixel(x, y, color, semi_transparent);
                }
            }
        }
    }

    fn polygon(&mut self, command: &[u32]) {
        let opcode = (command[0] >> 24) as u8;
        let gouraud = opcode & 0x10 != 0;
        let quad = opcode & 0x08 != 0;
        let textured = opcode & 0x04 != 0;
        let vertex_count = if quad { 4 } else { 3 };
        let mut vertices = [RenderVertex::default(); 4];
        let mut packet_index = 1usize;
        let mut color = Self::color8(command[0]);
        let mut clut = 0u16;
        let mut tpage = self.draw_mode;
        for (vertex_index, vertex) in vertices.iter_mut().enumerate().take(vertex_count) {
            if gouraud && vertex_index != 0 {
                color = Self::color8(command[packet_index]);
                packet_index += 1;
            }
            let (x, y) = self.vertex(command[packet_index]);
            packet_index += 1;
            let mut u = 0i32;
            let mut v = 0i32;
            if textured {
                let texture = command[packet_index];
                packet_index += 1;
                u = (texture & 0xff) as i32;
                v = ((texture >> 8) & 0xff) as i32;
                if vertex_index == 0 {
                    clut = (texture >> 16) as u16;
                } else if vertex_index == 1 {
                    tpage = (texture >> 16) as u16;
                }
            }
            *vertex = RenderVertex { x, y, color, u, v };
        }
        if textured {
            self.draw_mode = (self.draw_mode & !0x01ff) | (tpage & 0x01ff);
            tpage = (tpage & 0x01ff) | (self.draw_mode & !0x01ff);
        }
        self.render_triangle([vertices[0], vertices[1], vertices[2]], opcode, tpage, clut);
        if quad {
            self.render_triangle([vertices[1], vertices[2], vertices[3]], opcode, tpage, clut);
        }
    }

    fn render_line_segment(
        &mut self,
        mut start: RenderVertex,
        mut end: RenderVertex,
        semi_transparent: bool,
        gouraud: bool,
    ) {
        let original_dx = end.x - start.x;
        let original_dy = end.y - start.y;
        if original_dx.abs() > 1023 || original_dy.abs() > 511 {
            return;
        }
        let steps = original_dx.abs().max(original_dy.abs());
        let dither = self.draw_mode & (1 << 9) != 0;
        if steps == 0 {
            let color = Self::rgb8_to_15(start.color, start.x, start.y, dither);
            self.write_render_pixel(start.x, start.y, color, semi_transparent);
            return;
        }

        if start.x >= end.x {
            std::mem::swap(&mut start, &mut end);
        }
        let dx = end.x - start.x;
        let dy = end.y - start.y;
        const XY_SHIFT: u32 = 32;
        const RGB_SHIFT: u32 = 12;
        let divide_xy = |delta: i32| -> i64 {
            let rounding = if delta > 0 {
                i64::from(steps - 1)
            } else if delta < 0 {
                -i64::from(steps - 1)
            } else {
                0
            };
            ((i64::from(delta) << XY_SHIFT) + rounding) / i64::from(steps)
        };
        let dx_step = divide_xy(dx);
        let dy_step = divide_xy(dy);
        let mut x_fixed = (i64::from(start.x) << XY_SHIFT) + (1i64 << (XY_SHIFT - 1)) - 1024;
        let mut y_fixed = (i64::from(start.y) << XY_SHIFT) + (1i64 << (XY_SHIFT - 1));
        if dy_step < 0 {
            y_fixed -= 1024;
        }
        let mut rgb_fixed = [0i64; 3];
        let mut rgb_step = [0i64; 3];
        if gouraud {
            for channel in 0..3 {
                rgb_fixed[channel] =
                    (i64::from(start.color[channel]) << RGB_SHIFT) + (1 << (RGB_SHIFT - 1));
                rgb_step[channel] = (i64::from(end.color[channel] - start.color[channel])
                    << RGB_SHIFT)
                    / i64::from(steps);
            }
        }

        for _ in 0..=steps {
            let x = (x_fixed >> XY_SHIFT) as i32;
            let y = (y_fixed >> XY_SHIFT) as i32;
            let rgb = if gouraud {
                [
                    (rgb_fixed[0] >> RGB_SHIFT) as i32,
                    (rgb_fixed[1] >> RGB_SHIFT) as i32,
                    (rgb_fixed[2] >> RGB_SHIFT) as i32,
                ]
            } else {
                start.color
            };
            let color = Self::rgb8_to_15(rgb, x, y, dither);
            self.write_render_pixel(x, y, color, semi_transparent);
            x_fixed += dx_step;
            y_fixed += dy_step;
            if gouraud {
                for channel in 0..3 {
                    rgb_fixed[channel] += rgb_step[channel];
                }
            }
        }
    }

    fn line(&mut self, command: &[u32]) {
        let opcode = (command[0] >> 24) as u8;
        let gouraud = opcode & 0x10 != 0;
        let polyline = opcode & 0x08 != 0;
        let semi_transparent = opcode & 0x02 != 0;
        let mut vertices = Vec::new();
        let mut packet_index = 1usize;
        let mut color = Self::color8(command[0]);
        if packet_index >= command.len() {
            return;
        }
        let (x, y) = self.vertex(command[packet_index]);
        packet_index += 1;
        vertices.push(RenderVertex {
            x,
            y,
            color,
            ..RenderVertex::default()
        });

        while packet_index < command.len() {
            if gouraud {
                if command[packet_index] & 0xf000_f000 == 0x5000_5000 {
                    break;
                }
                color = Self::color8(command[packet_index]);
                packet_index += 1;
                if packet_index >= command.len() {
                    break;
                }
            } else if polyline && command[packet_index] & 0xf000_f000 == 0x5000_5000 {
                break;
            }
            let (x, y) = self.vertex(command[packet_index]);
            packet_index += 1;
            vertices.push(RenderVertex {
                x,
                y,
                color,
                ..RenderVertex::default()
            });
            if !polyline && vertices.len() == 2 {
                break;
            }
        }

        for segment in vertices.windows(2) {
            self.render_line_segment(segment[0], segment[1], semi_transparent, gouraud);
        }
    }

    fn rectangle(&mut self, command: &[u32]) {
        let opcode = (command[0] >> 24) as u8;
        let textured = opcode & 0x04 != 0;
        let semi_transparent = opcode & 0x02 != 0;
        let raw_texture = opcode & 0x01 != 0;
        let size_code = (opcode >> 3) & 3;
        let (x, y) = self.vertex(command[1]);
        let mut packet_index = 2usize;
        let mut u = 0i32;
        let mut v = 0i32;
        let mut clut = 0u16;
        if textured {
            let texture = command[packet_index];
            packet_index += 1;
            u = (texture & 0xff) as i32;
            v = ((texture >> 8) & 0xff) as i32;
            clut = (texture >> 16) as u16;
        }
        let (width, height) = match size_code {
            0 => {
                let (width, height) = Self::wh(command[packet_index]);
                (i32::from(width & 0x03ff), i32::from(height & 0x01ff))
            }
            1 => (1, 1),
            2 => (8, 8),
            _ => (16, 16),
        };
        let color_rgb = Self::color8(command[0]);
        let flat_color = Self::color15(command[0]);
        if textured {
            self.prepare_clut_cache(self.draw_mode, clut);
        }
        let x_flip = self.draw_mode & (1 << 12) != 0;
        let y_flip = self.draw_mode & (1 << 13) != 0;
        for dy in 0..height {
            for dx in 0..width {
                if textured {
                    let tex_x = if x_flip { -dx } else { dx };
                    let tex_y = if y_flip { -dy } else { dy };
                    let texture = self.texture_pixel(self.draw_mode, clut, u + tex_x, v + tex_y);
                    if texture == 0 {
                        continue;
                    }
                    let color = if raw_texture {
                        texture
                    } else {
                        Self::modulate_texture(texture, color_rgb, x + dx, y + dy, false)
                    };
                    self.write_render_pixel(
                        x + dx,
                        y + dy,
                        color,
                        semi_transparent && texture & 0x8000 != 0,
                    );
                } else {
                    self.write_render_pixel(x + dx, y + dy, flat_color, semi_transparent);
                }
            }
        }
    }

    fn copy_vram(&mut self, command: &[u32]) {
        let (sx, sy) = Self::xy(command[1]);
        let (dx, dy) = Self::xy(command[2]);
        let (width, height) = Self::wh(command[3]);
        let width = Self::normalized_extent(width, 0x03ff);
        let height = Self::normalized_extent(height, 0x01ff);
        let mut row = Vec::with_capacity(usize::from(width));
        for y in 0..i32::from(height) {
            row.clear();
            for x in 0..i32::from(width) {
                row.push(self.vram[Self::vram_index(sx + x, sy + y)]);
            }
            for (x, color) in row.iter().copied().enumerate() {
                self.write_transfer_pixel(dx + x as i32, dy + y, color);
            }
        }
    }
    fn normalized_extent(value: u16, mask: u16) -> u16 {
        let value = value & mask;
        if value == 0 {
            mask.wrapping_add(1)
        } else {
            value
        }
    }

    fn transfer_from_words(position: u32, size: u32) -> Transfer {
        let (x, y) = Self::xy(position);
        let (width, height) = Self::wh(size);
        let width = Self::normalized_extent(width, 0x03ff);
        let height = Self::normalized_extent(height, 0x01ff);
        Transfer {
            x: x.rem_euclid(VRAM_WIDTH as i32) as u16,
            y: y.rem_euclid(VRAM_HEIGHT as i32) as u16,
            width,
            height,
            index: 0,
            total: u32::from(width) * u32::from(height),
        }
    }

    fn begin_cpu_to_vram(&mut self, command: &[u32]) {
        self.transfer_write = Some(Self::transfer_from_words(command[1], command[2]));
    }

    fn begin_vram_to_cpu(&mut self, command: &[u32]) {
        self.transfer_read = Some(Self::transfer_from_words(command[1], command[2]));
    }

    fn write_transfer_half(&mut self, transfer: &mut Transfer, value: u16) {
        if transfer.index >= transfer.total {
            return;
        }
        let x = u32::from(transfer.x) + transfer.index % u32::from(transfer.width);
        let y = u32::from(transfer.y) + transfer.index / u32::from(transfer.width);
        self.write_transfer_pixel(x as i32, y as i32, value);
        transfer.index += 1;
    }
    pub fn gpuread(&mut self) -> u32 {
        let Some(mut transfer) = self.transfer_read.take() else {
            return self.read_latch;
        };
        let lo = self.read_transfer_half(&mut transfer);
        let hi = if transfer.index < transfer.total {
            self.read_transfer_half(&mut transfer)
        } else {
            0
        };
        if transfer.index < transfer.total {
            self.transfer_read = Some(transfer);
        }
        self.refresh_status();
        u32::from(lo) | (u32::from(hi) << 16)
    }

    fn read_transfer_half(&self, transfer: &mut Transfer) -> u16 {
        if transfer.index >= transfer.total {
            return 0;
        }
        let x = u32::from(transfer.x) + transfer.index % u32::from(transfer.width);
        let y = u32::from(transfer.y) + transfer.index / u32::from(transfer.width);
        transfer.index += 1;
        self.vram[Self::vram_index(x as i32, y as i32)]
    }

    pub fn gp1(&mut self, value: u32) {
        match (value >> 24) as u8 {
            0x00 => self.gpu_reset(),
            0x01 => {
                self.command.clear();
                self.fifo.clear();
                self.transfer_write = None;
                self.transfer_read = None;
                self.pending_command_ticks = 0;
                self.clut_cache_len = 0;
            }
            0x02 => {
                self.irq = false;
                self.status &= !(1 << 24);
            }
            0x03 => self.display_disabled = value & 1 != 0,
            0x04 => self.dma_direction = (value & 3) as u8,
            0x05 => {
                self.display_x = (value & 0x03ff) as u16;
                self.display_y = ((value >> 10) & 0x01ff) as u16;
            }
            0x06 => {
                self.display_h_start = (value & 0x0fff) as u16;
                self.display_h_end = ((value >> 12) & 0x0fff) as u16;
            }
            0x07 => {
                self.display_v_start = (value & 0x03ff) as u16;
                self.display_v_end = ((value >> 10) & 0x03ff) as u16;
            }
            0x08 => self.set_display_mode(value),
            0x10..=0x1f => self.latch_gpu_info(value),
            _ => {}
        }
        self.refresh_status();
    }

    fn latch_gpu_info(&mut self, value: u32) {
        match value & 0x0f {
            2 => self.read_latch = self.texture_window,
            3 => {
                self.read_latch =
                    u32::from(self.draw_area_min.0) | (u32::from(self.draw_area_min.1) << 10)
            }
            4 => {
                self.read_latch =
                    u32::from(self.draw_area_max.0) | (u32::from(self.draw_area_max.1) << 10)
            }
            5 => {
                self.read_latch = u32::from(self.draw_offset.0 as u16 & 0x07ff)
                    | (u32::from(self.draw_offset.1 as u16 & 0x07ff) << 11)
            }
            7 => self.read_latch = 2,
            8 => self.read_latch = 0,
            _ => {}
        }
    }

    fn gpu_reset(&mut self) {
        self.command.clear();
        self.fifo.clear();
        self.transfer_write = None;
        self.transfer_read = None;
        self.display_x = 0;
        self.display_y = 0;
        self.display_h_start = 0x0200;
        self.display_h_end = 0x0c00;
        self.display_v_start = 0x0010;
        self.display_v_end = 0x0100;
        self.display_mode = 0;
        self.display_disabled = true;
        self.dma_direction = 0;
        self.draw_mode = 0;
        self.texture_window = 0;
        self.clut_cache_len = 0;
        self.draw_area_min = (0, 0);
        self.draw_area_max = (1023, 511);
        self.draw_offset = (0, 0);
        self.force_mask = false;
        self.check_mask = false;
        self.read_latch = 0;
        self.status = 0x1480_2000;
        self.pending_command_ticks = 0;
        self.irq = false;
    }

    fn set_display_mode(&mut self, value: u32) {
        self.display_mode = (value & 0xff) as u8;
    }

    pub fn frame_rate(&self) -> f64 {
        if self.display_mode & 0x08 != 0 {
            50.0
        } else {
            59.94
        }
    }

    pub fn cpu_cycles_per_frame(&self) -> u64 {
        if self.display_mode & 0x08 != 0 {
            677_376
        } else {
            564_480
        }
    }

    pub fn video_clock_hz(&self) -> u64 {
        if self.display_mode & 0x08 != 0 {
            53_203_425
        } else {
            53_693_175
        }
    }

    pub fn video_cycles_per_scanline(&self) -> u64 {
        if self.display_mode & 0x08 != 0 {
            3_406
        } else {
            3_413
        }
    }

    fn scanlines_per_frame(&self) -> u64 {
        if self.display_mode & 0x08 != 0 {
            314
        } else {
            263
        }
    }

    pub fn dot_clock_divisor(&self) -> u64 {
        u64::from(self.horizontal_resolution().1)
    }

    fn timing_position(&self) -> (u64, u64) {
        let cycles_per_frame = self.cpu_cycles_per_frame();
        let scanlines = self.scanlines_per_frame();
        let scaled = self.cycle_phase.saturating_mul(scanlines);
        let scanline = scaled / cycles_per_frame;
        let line_fraction = scaled % cycles_per_frame;
        let video_cycle =
            line_fraction.saturating_mul(self.video_cycles_per_scanline()) / cycles_per_frame;
        (scanline.min(scanlines - 1), video_cycle)
    }

    pub fn in_hblank(&self) -> bool {
        let (_, video_cycle) = self.timing_position();
        let start = u64::from(self.display_h_start);
        let end = u64::from(self.display_h_end);
        if start == end {
            true
        } else if start < end {
            !(start..end).contains(&video_cycle)
        } else {
            video_cycle >= end && video_cycle < start
        }
    }

    pub fn in_vblank(&self) -> bool {
        let (scanline, _) = self.timing_position();
        let start = u64::from(self.display_v_start);
        let end = u64::from(self.display_v_end);
        if start == end {
            true
        } else if start < end {
            !(start..end).contains(&scanline)
        } else {
            scanline >= end && scanline < start
        }
    }

    fn horizontal_resolution(&self) -> (u32, u16) {
        if self.display_mode & 0x40 != 0 {
            (368, 7)
        } else {
            match self.display_mode & 3 {
                0 => (256, 10),
                1 => (320, 8),
                2 => (512, 5),
                _ => (640, 4),
            }
        }
    }

    fn output_dimensions(&self) -> (u32, u32) {
        let (nominal_width, clocks_per_pixel) = self.horizontal_resolution();
        let clocks = u32::from(self.display_h_end.wrapping_sub(self.display_h_start) & 0x0fff);
        let mut width = if clocks == 0 {
            nominal_width
        } else {
            ((clocks / u32::from(clocks_per_pixel)) + 2) & !3
        };
        if width == 0 {
            width = nominal_width;
        }
        width = width.clamp(4, 1024);

        let vertical_lines =
            u32::from(self.display_v_end.wrapping_sub(self.display_v_start) & 0x03ff);
        let mut height = if vertical_lines == 0 {
            240
        } else {
            vertical_lines
        };
        if self.display_mode & 0x24 == 0x24 {
            height = height.saturating_mul(2);
        }
        (width, height.clamp(1, 512))
    }

    fn refresh_status(&mut self) {
        self.status = (self.status & !0x0000_1fff)
            | u32::from(self.draw_mode & 0x07ff)
            | (u32::from(self.force_mask) << 11)
            | (u32::from(self.check_mask) << 12);
        if self.draw_mode & (1 << 11) != 0 {
            self.status |= 1 << 15;
        } else {
            self.status &= !(1 << 15);
        }
        self.status &= !((1 << 13) | (1 << 14) | (1 << 16) | (0x3f << 17) | (1 << 31));
        if self.display_mode & 0x20 == 0 || self.frame & 1 == 0 {
            self.status |= 1 << 13;
        }
        self.status |= u32::from((self.display_mode >> 7) & 1) << 14;
        self.status |= u32::from((self.display_mode >> 6) & 1) << 16;
        self.status |= u32::from(self.display_mode & 0x3f) << 17;
        if !self.in_vblank() {
            let line_lsb = if self.interleaved_480i_mode() {
                self.active_line_lsb()
            } else {
                ((u64::from(self.display_y) + self.timing_position().0) & 1) as u8
            };
            self.status |= u32::from(line_lsb) << 31;
        }
        if self.display_disabled {
            self.status |= 1 << 23;
        } else {
            self.status &= !(1 << 23);
        }
        self.status = (self.status & !(3 << 29)) | (u32::from(self.dma_direction) << 29);
        if self.irq {
            self.status |= 1 << 24;
        } else {
            self.status &= !(1 << 24);
        }
        let transfer_write_active = self.transfer_write.is_some();
        let ready_for_vram_read = self.transfer_read.is_some();
        let transfer_active = transfer_write_active || ready_for_vram_read;
        let gpu_busy = self.pending_command_ticks != 0;
        let fifo_not_full = self.fifo_occupancy() < GP0_FIFO_WORDS;
        let ready_for_command =
            self.command.is_empty() && !transfer_active && !gpu_busy && fifo_not_full;
        let packet_blocks_dma = self.command.first().is_some_and(|word| {
            let opcode = (word >> 24) as u8;
            (0x20..=0x5f).contains(&opcode)
        });
        let ready_for_dma_block = if !fifo_not_full {
            false
        } else if transfer_write_active {
            true
        } else if ready_for_vram_read || gpu_busy {
            false
        } else if self.command.is_empty() {
            true
        } else {
            !packet_blocks_dma
        };
        self.status &= !((1 << 25) | (1 << 26) | (1 << 27) | (1 << 28));
        self.status |= u32::from(ready_for_command) << 26;
        self.status |= u32::from(ready_for_vram_read) << 27;
        self.status |= u32::from(ready_for_dma_block) << 28;
        let dma_request = match self.dma_direction {
            1 => fifo_not_full,
            2 => ready_for_dma_block,
            3 => ready_for_vram_read,
            _ => false,
        };
        self.status |= u32::from(dma_request) << 25;
    }

    pub fn status(&mut self) -> u32 {
        self.refresh_status();
        self.status
    }

    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        let mut gpu_ticks = u64::from(cycles).saturating_mul(2);
        self.process_gp0_fifo();
        while gpu_ticks != 0 && self.pending_command_ticks != 0 {
            if gpu_ticks < self.pending_command_ticks {
                self.pending_command_ticks -= gpu_ticks;
                gpu_ticks = 0;
            } else {
                gpu_ticks -= self.pending_command_ticks;
                self.pending_command_ticks = 0;
                self.process_gp0_fifo();
            }
        }
        self.refresh_status();
        let cycles_per_frame = self.cpu_cycles_per_frame();
        let scanlines = self.scanlines_per_frame();
        let elapsed = u64::from(cycles);
        let old_phase = self.cycle_phase;
        let end_scanline = u64::from(self.display_v_end);
        let vblank_phase =
            if self.display_v_start == self.display_v_end || end_scanline >= scanlines {
                0
            } else {
                end_scanline
                    .saturating_mul(cycles_per_frame)
                    .div_ceil(scanlines)
                    .min(cycles_per_frame - 1)
            };
        let total_phase = old_phase.saturating_add(elapsed);
        let first_vblank = if old_phase < vblank_phase {
            vblank_phase
        } else {
            vblank_phase.saturating_add(cycles_per_frame)
        };
        if total_phase >= first_vblank {
            let crossings = 1 + (total_phase - first_vblank) / cycles_per_frame;
            for _ in 0..crossings {
                self.render_display();
                self.frame = self.frame.wrapping_add(1);
            }
        }
        self.cycle_phase = total_phase % cycles_per_frame;
    }

    fn vram_byte(&self, byte_address: usize) -> u8 {
        let byte_address = byte_address % (self.vram.len() * 2);
        let value = self.vram[byte_address / 2];
        if byte_address & 1 == 0 {
            value as u8
        } else {
            (value >> 8) as u8
        }
    }

    fn render_display(&mut self) {
        let (width, height) = self.output_dimensions();
        if self.video.width() != width || self.video.height() != height {
            self.video.resize(width, height);
        }
        if self.display_disabled {
            self.video.clear([0, 0, 0, 255]);
            return;
        }
        let display_x = usize::from(self.display_x);
        let display_y = usize::from(self.display_y);
        let twenty_four_bit = self.display_mode & 0x10 != 0;
        for y in 0..height as usize {
            for x in 0..width as usize {
                let rgba = if twenty_four_bit {
                    let byte_address =
                        ((display_y + y) % VRAM_HEIGHT) * VRAM_WIDTH * 2 + display_x * 2 + x * 3;
                    [
                        self.vram_byte(byte_address),
                        self.vram_byte(byte_address + 1),
                        self.vram_byte(byte_address + 2),
                        255,
                    ]
                } else {
                    let value = self.vram[((display_y + y) % VRAM_HEIGHT) * VRAM_WIDTH
                        + ((display_x + x) % VRAM_WIDTH)];
                    [
                        ((value & 0x1f) * 255 / 31) as u8,
                        (((value >> 5) & 0x1f) * 255 / 31) as u8,
                        (((value >> 10) & 0x1f) * 255 / 31) as u8,
                        255,
                    ]
                };
                let offset = (y * width as usize + x) * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    pub fn request_irq(&mut self) {
        self.irq = true;
        self.status |= 1 << 24;
    }
    pub fn save(&self, out: &mut StateWriter) {
        out.u32(self.vram.len() as u32);
        for pixel in &self.vram {
            out.u16(*pixel);
        }
        out.u32(self.command.len() as u32);
        for word in &self.command {
            out.u32(*word);
        }
        out.u32(self.fifo.len() as u32);
        for word in &self.fifo {
            out.u32(*word);
        }
        Self::save_transfer(out, self.transfer_write);
        Self::save_transfer(out, self.transfer_read);
        out.u16(self.display_x);
        out.u16(self.display_y);
        out.u16(self.display_h_start);
        out.u16(self.display_h_end);
        out.u16(self.display_v_start);
        out.u16(self.display_v_end);
        out.u8(self.display_mode);
        out.u8(self.display_disabled as u8);
        out.u8(self.dma_direction);
        out.u16(self.draw_mode);
        out.u32(self.texture_window);
        out.u16(self.clut_cache_key);
        out.u16(self.clut_cache_len);
        for color in self.clut_cache {
            out.u16(color);
        }
        out.u16(self.draw_area_min.0);
        out.u16(self.draw_area_min.1);
        out.u16(self.draw_area_max.0);
        out.u16(self.draw_area_max.1);
        out.u16(self.draw_offset.0 as u16);
        out.u16(self.draw_offset.1 as u16);
        out.u8(self.force_mask as u8);
        out.u8(self.check_mask as u8);
        out.u32(self.read_latch);
        out.u32(self.status);
        out.u64(self.frame);
        out.u64(self.cycle_phase);
        out.u64(self.pending_command_ticks);
        out.u8(self.irq as u8);
    }

    fn save_transfer(out: &mut StateWriter, transfer: Option<Transfer>) {
        match transfer {
            Some(t) => {
                out.u8(1);
                out.u16(t.x);
                out.u16(t.y);
                out.u16(t.width);
                out.u16(t.height);
                out.u32(t.index);
                out.u32(t.total);
            }
            None => out.u8(0),
        }
    }

    fn load_transfer(input: &mut StateReader<'_>) -> Result<Option<Transfer>, String> {
        if input.u8()? == 0 {
            return Ok(None);
        }
        Ok(Some(Transfer {
            x: input.u16()?,
            y: input.u16()?,
            width: input.u16()?,
            height: input.u16()?,
            index: input.u32()?,
            total: input.u32()?,
        }))
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let len = input.u32()? as usize;
        if len != VRAM_WIDTH * VRAM_HEIGHT {
            return Err("invalid PlayStation VRAM state length".into());
        }
        for pixel in &mut self.vram {
            *pixel = input.u16()?;
        }
        let command_len = input.u32()? as usize;
        if command_len > 64 {
            return Err("invalid PlayStation GPU command state length".into());
        }
        self.command.clear();
        for _ in 0..command_len {
            self.command.push(input.u32()?);
        }
        let fifo_len = input.u32()? as usize;
        if fifo_len > GP0_FIFO_WORDS {
            return Err("invalid PlayStation GPU FIFO state length".into());
        }
        self.fifo.clear();
        for _ in 0..fifo_len {
            self.fifo.push_back(input.u32()?);
        }
        self.transfer_write = Self::load_transfer(input)?;
        self.transfer_read = Self::load_transfer(input)?;
        self.display_x = input.u16()? & 0x03ff;
        self.display_y = input.u16()? & 0x01ff;
        self.display_h_start = input.u16()? & 0x0fff;
        self.display_h_end = input.u16()? & 0x0fff;
        self.display_v_start = input.u16()? & 0x03ff;
        self.display_v_end = input.u16()? & 0x03ff;
        self.display_mode = input.u8()?;
        self.display_disabled = input.u8()? != 0;
        self.dma_direction = input.u8()? & 3;
        self.draw_mode = input.u16()? & 0x3fff;
        self.texture_window = input.u32()? & 0x000f_ffff;
        self.clut_cache_key = input.u16()? & 0x7fff;
        self.clut_cache_len = input.u16()?;
        if self.clut_cache_len > 256 {
            return Err("invalid PlayStation GPU CLUT cache length".into());
        }
        for color in &mut self.clut_cache {
            *color = input.u16()?;
        }
        self.draw_area_min = (input.u16()? & 0x03ff, input.u16()? & 0x01ff);
        self.draw_area_max = (input.u16()? & 0x03ff, input.u16()? & 0x01ff);
        self.draw_offset = (input.u16()? as i16, input.u16()? as i16);
        self.force_mask = input.u8()? != 0;
        self.check_mask = input.u8()? != 0;
        self.read_latch = input.u32()?;
        self.status = input.u32()?;
        self.frame = input.u64()?;
        self.cycle_phase = input.u64()? % self.cpu_cycles_per_frame();
        self.pending_command_ticks = input.u64()?;
        self.irq = input.u8()? != 0;
        self.refresh_status();
        self.render_display();
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn vram_pixel(&self, x: usize, y: usize) -> u16 {
        self.vram[(y % VRAM_HEIGHT) * VRAM_WIDTH + (x % VRAM_WIDTH)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn complete_gpu_commands(gpu: &mut Ps1Gpu) {
        for _ in 0..1_000_000 {
            if gpu.pending_command_ticks == 0
                && gpu.fifo.is_empty()
                && gpu.command.is_empty()
                && gpu.transfer_write.is_none()
            {
                return;
            }
            gpu.tick_cpu_cycles(1);
        }
        panic!("PlayStation GPU command queue did not drain");
    }

    #[test]
    fn fill_rectangle_reaches_vram_and_display() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x0200_00ff);
        gpu.gp0(0);
        gpu.gp0((16 << 16) | 16);
        assert_ne!(gpu.vram_pixel(0, 0), 0);
        gpu.gp1(0x0300_0000);
        gpu.tick_cpu_cycles(564_480);
        assert!(gpu.video().pixels()[0] > 200);
        assert_eq!(gpu.frame(), 1);
    }

    #[test]
    fn fill_rectangle_masks_aligns_and_obeys_interlace_field_suppression() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp1(0x0800_0024);
        gpu.gp0(0x0200_ff00);
        gpu.gp0((0x0201 << 16) | 0x001f);
        gpu.gp0((2 << 16) | 1);

        for x in 0x10..0x20 {
            assert_eq!(gpu.vram_pixel(x, 1), 0x03e0);
            assert_eq!(gpu.vram_pixel(x, 2), 0);
        }
        assert_eq!(gpu.vram_pixel(0x0f, 1), 0);
        assert_eq!(gpu.vram_pixel(0x20, 1), 0);

        gpu.gp0(0x0200_00ff);
        gpu.gp0(0);
        gpu.gp0(1 << 16);
        assert_eq!(gpu.vram_pixel(0, 0), 0);
    }

    #[test]
    fn fill_rectangle_timing_masks_size_fields_like_the_pixel_path() {
        let gpu = Ps1Gpu::new();
        let command = [0x0200_0000, 0, 0xffff_ffff];
        let expected = 46 + (1024 / 8 + 9) * 511;
        assert_eq!(gpu.command_execution_ticks(&command), expected);
    }

    #[test]
    fn fill_rectangle_rounds_maximum_width_to_full_vram_row() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x02ff_0000);
        gpu.gp0((3 << 16) | 0x03ff);
        gpu.gp0((1 << 16) | 0x03f1);
        assert_ne!(gpu.vram_pixel(0x03f0, 3), 0);
        assert_ne!(gpu.vram_pixel(0x0000, 3), 0);
        assert_ne!(gpu.vram_pixel(0x03ef, 3), 0);
    }

    #[test]
    fn gp1_command_and_gpu_resets_clear_clut_cache() {
        let mut gpu = Ps1Gpu::new();
        gpu.clut_cache_len = 16;
        gpu.gp1(0x0100_0000);
        assert_eq!(gpu.clut_cache_len, 0);

        gpu.clut_cache_len = 256;
        gpu.gp1(0x0000_0000);
        assert_eq!(gpu.clut_cache_len, 0);
    }

    #[test]
    fn cpu_vram_transfer_round_trips_pixels() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0xa000_0000);
        gpu.gp0((20 << 16) | 10);
        gpu.gp0((1 << 16) | 2);
        gpu.gp0(0x1234_7fff);
        assert_eq!(gpu.vram_pixel(10, 20), 0x7fff);
        assert_eq!(gpu.vram_pixel(11, 20), 0x1234);
        gpu.gp0(0xc000_0000);
        gpu.gp0((20 << 16) | 10);
        gpu.gp0((1 << 16) | 2);
        assert_eq!(gpu.gpuread(), 0x1234_7fff);
    }
    #[test]
    fn vram_copy_buffers_each_row_but_propagates_downward_overlap() {
        let mut gpu = Ps1Gpu::new();
        gpu.vram[Ps1Gpu::vram_index(10, 10)] = 0x0011;
        gpu.vram[Ps1Gpu::vram_index(11, 10)] = 0x0012;
        gpu.vram[Ps1Gpu::vram_index(10, 11)] = 0x0021;
        gpu.vram[Ps1Gpu::vram_index(11, 11)] = 0x0022;
        gpu.vram[Ps1Gpu::vram_index(10, 12)] = 0x0031;
        gpu.vram[Ps1Gpu::vram_index(11, 12)] = 0x0032;
        gpu.gp0(0x8000_0000);
        gpu.gp0((10 << 16) | 10);
        gpu.gp0((11 << 16) | 10);
        gpu.gp0((3 << 16) | 2);
        complete_gpu_commands(&mut gpu);
        for y in 11..=13 {
            assert_eq!(gpu.vram_pixel(10, y), 0x0011);
            assert_eq!(gpu.vram_pixel(11, y), 0x0012);
        }

        gpu.vram[Ps1Gpu::vram_index(20, 20)] = 0x0101;
        gpu.vram[Ps1Gpu::vram_index(21, 20)] = 0x0102;
        gpu.vram[Ps1Gpu::vram_index(22, 20)] = 0x0103;
        gpu.gp0(0x8000_0000);
        gpu.gp0((20 << 16) | 20);
        gpu.gp0((20 << 16) | 21);
        gpu.gp0((1 << 16) | 3);
        complete_gpu_commands(&mut gpu);
        assert_eq!(gpu.vram_pixel(21, 20), 0x0101);
        assert_eq!(gpu.vram_pixel(22, 20), 0x0102);
        assert_eq!(gpu.vram_pixel(23, 20), 0x0103);
    }

    #[test]
    fn flat_triangle_rasterizes_inside_bounds() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x2000_ff00);
        gpu.gp0((10 << 16) | 10);
        gpu.gp0((10 << 16) | 30);
        gpu.gp0((30 << 16) | 10);
        assert_ne!(gpu.vram_pixel(15, 15), 0);
        assert_eq!(gpu.vram_pixel(40, 40), 0);
    }

    #[test]
    fn polygon_excludes_lower_right_edges_for_both_windings() {
        for vertices in [
            [(10u32, 10u32), (10, 14), (14, 10)],
            [(10u32, 10u32), (14, 10), (10, 14)],
        ] {
            let mut gpu = Ps1Gpu::new();
            gpu.gp0(0x2000_00ff);
            for (x, y) in vertices {
                gpu.gp0((y << 16) | x);
            }
            assert_ne!(gpu.vram_pixel(10, 10), 0);
            assert_ne!(gpu.vram_pixel(13, 10), 0);
            assert_ne!(gpu.vram_pixel(10, 13), 0);
            assert_ne!(gpu.vram_pixel(11, 11), 0);
            assert_eq!(gpu.vram_pixel(14, 10), 0);
            assert_eq!(gpu.vram_pixel(10, 14), 0);
            assert_eq!(gpu.vram_pixel(12, 12), 0);
        }
    }

    #[test]
    fn four_bit_clut_texture_maps_onto_triangle() {
        let mut gpu = Ps1Gpu::new();
        let clut = (20u32 << 6) | 1;
        gpu.vram[Ps1Gpu::vram_index(0, 10)] = 1;
        gpu.vram[Ps1Gpu::vram_index(17, 20)] = 0x03e0;
        gpu.gp0(0x2500_0000);
        gpu.gp0((20 << 16) | 20);
        gpu.gp0((clut << 16) | (10 << 8));
        gpu.gp0((20 << 16) | 30);
        gpu.gp0(10 << 8);
        gpu.gp0((30 << 16) | 20);
        gpu.gp0(10 << 8);
        assert_eq!(gpu.vram_pixel(22, 22) & 0x7fff, 0x03e0);
    }

    #[test]
    fn texture_window_repeats_selected_texel_bits() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0xe100_0100);
        gpu.gp0(0xe200_0401);
        gpu.vram[Ps1Gpu::vram_index(8, 0)] = 0x001f;
        gpu.gp0(0x6500_0000);
        gpu.gp0((40 << 16) | 40);
        gpu.gp0(0);
        gpu.gp0((1 << 16) | 1);
        complete_gpu_commands(&mut gpu);
        assert_eq!(gpu.vram_pixel(40, 40) & 0x7fff, 0x001f);
    }

    #[test]
    fn modulated_texture_uses_polygon_dither_matrix() {
        let texture = 0x4210;
        let rgb = [128, 128, 128];
        assert_eq!(Ps1Gpu::modulate_texture(texture, rgb, 0, 0, false), 0x4210);
        assert_eq!(Ps1Gpu::modulate_texture(texture, rgb, 0, 0, true), 0x3def);
        assert_eq!(Ps1Gpu::modulate_texture(texture, rgb, 1, 0, true), 0x4210);
    }

    #[test]
    fn raw_fifteen_bit_texture_rectangle_preserves_texel() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0xe100_0100);
        gpu.vram[Ps1Gpu::vram_index(4, 5)] = 0x7c1f;
        gpu.gp0(0x6500_0000);
        gpu.gp0((80 << 16) | 80);
        gpu.gp0((5 << 8) | 4);
        gpu.gp0((2 << 16) | 2);
        complete_gpu_commands(&mut gpu);
        assert_eq!(gpu.vram_pixel(80, 80) & 0x7fff, 0x7c1f);
    }

    #[test]
    fn clut_cache_reuses_palette_until_clear_or_expansion() {
        let mut gpu = Ps1Gpu::new();
        let clut = 20u16 << 6;
        gpu.gp0(0xe100_0001);
        gpu.vram[Ps1Gpu::vram_index(64, 0)] = 0x0001;
        gpu.vram[Ps1Gpu::vram_index(1, 20)] = 0x001f;

        let draw = |gpu: &mut Ps1Gpu, x: u32| {
            gpu.gp0(0x6500_0000);
            gpu.gp0((100 << 16) | x);
            gpu.gp0(u32::from(clut) << 16);
            gpu.gp0((1 << 16) | 1);
            complete_gpu_commands(gpu);
        };
        draw(&mut gpu, 100);
        assert_eq!(gpu.vram_pixel(100, 100) & 0x7fff, 0x001f);

        gpu.vram[Ps1Gpu::vram_index(1, 20)] = 0x03e0;
        draw(&mut gpu, 101);
        assert_eq!(gpu.vram_pixel(101, 100) & 0x7fff, 0x001f);

        gpu.gp0(0x0100_0000);
        complete_gpu_commands(&mut gpu);
        draw(&mut gpu, 102);
        assert_eq!(gpu.vram_pixel(102, 100) & 0x7fff, 0x03e0);

        gpu.vram[Ps1Gpu::vram_index(1, 20)] = 0x7c00;
        gpu.gp0(0xe100_0081);
        complete_gpu_commands(&mut gpu);
        draw(&mut gpu, 103);
        assert_eq!(gpu.vram_pixel(103, 100) & 0x7fff, 0x7c00);
    }

    #[test]
    fn sprite_texture_flip_decrements_from_the_supplied_origin() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0xe100_3100);
        gpu.vram[Ps1Gpu::vram_index(0, 0)] = 0x001f;
        gpu.vram[Ps1Gpu::vram_index(255, 0)] = 0x03e0;
        gpu.vram[Ps1Gpu::vram_index(254, 0)] = 0x7c00;
        gpu.vram[Ps1Gpu::vram_index(0, 255)] = 0x7c1f;
        gpu.vram[Ps1Gpu::vram_index(255, 255)] = 0x03ff;
        gpu.gp0(0x6500_0000);
        gpu.gp0((80 << 16) | 80);
        gpu.gp0(0);
        gpu.gp0((2 << 16) | 3);
        complete_gpu_commands(&mut gpu);
        assert_eq!(gpu.vram_pixel(80, 80) & 0x7fff, 0x001f);
        assert_eq!(gpu.vram_pixel(81, 80) & 0x7fff, 0x03e0);
        assert_eq!(gpu.vram_pixel(82, 80) & 0x7fff, 0x7c00);
        assert_eq!(gpu.vram_pixel(80, 81) & 0x7fff, 0x7c1f);
        assert_eq!(gpu.vram_pixel(81, 81) & 0x7fff, 0x03ff);
    }

    #[test]
    fn gouraud_triangle_interpolates_vertex_colors() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x3000_00ff);
        gpu.gp0((100 << 16) | 100);
        gpu.gp0(0x0000_ff00);
        gpu.gp0((100 << 16) | 120);
        gpu.gp0(0x00ff_0000);
        gpu.gp0((120 << 16) | 100);
        let pixel = gpu.vram_pixel(106, 106);
        assert_ne!(pixel & 0x001f, 0);
        assert_ne!(pixel & 0x03e0, 0);
        assert_ne!(pixel & 0x7c00, 0);
    }

    #[test]
    fn gouraud_triangle_uses_quantized_twelve_bit_attribute_gradients() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x3000_0000);
        gpu.gp0((100 << 16) | 100);
        gpu.gp0(0x0000_001b);
        gpu.gp0((100 << 16) | 107);
        gpu.gp0(0x0000_0000);
        gpu.gp0((107 << 16) | 100);
        assert_eq!(gpu.vram_pixel(102, 101) & 0x001f, 1);
    }

    #[test]
    fn drawing_offset_and_area_clip_render_primitives() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0xe300_0000 | 15 | (16 << 10));
        gpu.gp0(0xe400_0000 | 15 | (16 << 10));
        gpu.gp0(0xe500_0000 | 5 | (6 << 11));
        gpu.gp0(0x6800_00ff);
        gpu.gp0((10 << 16) | 10);
        complete_gpu_commands(&mut gpu);
        assert_ne!(gpu.vram_pixel(15, 16), 0);
        assert_eq!(gpu.vram_pixel(14, 16), 0);
    }

    #[test]
    fn semi_transparency_average_truncates_each_component_before_addition() {
        assert_eq!(Ps1Gpu::blend_pixel(0x0421, 0x0421, 0), 0);
        assert_eq!(Ps1Gpu::blend_pixel(0x0842, 0x0421, 0), 0x0421);
    }

    #[test]
    fn semi_transparent_rectangle_uses_selected_blend_mode() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0xe100_0020);
        gpu.vram[Ps1Gpu::vram_index(140, 140)] = 0x03e0;
        gpu.gp0(0x6200_00ff);
        gpu.gp0((140 << 16) | 140);
        gpu.gp0((1 << 16) | 1);
        complete_gpu_commands(&mut gpu);
        let pixel = gpu.vram_pixel(140, 140);
        assert_eq!(pixel & 0x001f, 0x001f);
        assert_eq!(pixel & 0x03e0, 0x03e0);
    }

    #[test]
    fn mask_bits_protect_render_and_transfer_destinations() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0xe600_0001);
        gpu.gp0(0x6800_00ff);
        gpu.gp0((160 << 16) | 160);
        complete_gpu_commands(&mut gpu);
        assert_ne!(gpu.vram_pixel(160, 160) & 0x8000, 0);

        gpu.gp0(0xe600_0002);
        gpu.gp0(0x6800_ff00);
        gpu.gp0((160 << 16) | 160);
        complete_gpu_commands(&mut gpu);
        assert_eq!(gpu.vram_pixel(160, 160) & 0x001f, 0x001f);

        gpu.gp0(0xa000_0000);
        gpu.gp0((160 << 16) | 160);
        gpu.gp0((1 << 16) | 1);
        gpu.gp0(0x0000_7c00);
        complete_gpu_commands(&mut gpu);
        assert_eq!(gpu.vram_pixel(160, 160) & 0x001f, 0x001f);
    }

    #[test]
    fn single_and_polyline_packets_render_without_desynchronizing_gp0() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x4000_00ff);
        gpu.gp0((180 << 16) | 180);
        gpu.gp0((180 << 16) | 190);
        assert_ne!(gpu.vram_pixel(185, 180) & 0x001f, 0);

        gpu.gp0(0x4800_00ff);
        gpu.gp0((190 << 16) | 180);
        gpu.gp0((200 << 16) | 180);
        gpu.gp0((200 << 16) | 190);
        gpu.gp0(0x5000_5000);
        complete_gpu_commands(&mut gpu);
        assert_ne!(gpu.vram_pixel(180, 195) & 0x001f, 0);
        assert_ne!(gpu.vram_pixel(185, 200) & 0x001f, 0);

        gpu.gp0(0x6800_ff00);
        gpu.gp0((210 << 16) | 210);
        complete_gpu_commands(&mut gpu);
        assert_ne!(gpu.vram_pixel(210, 210) & 0x03e0, 0);
    }

    #[test]
    fn negative_slope_line_uses_fixed_point_dda_pixel_choice() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x4000_00ff);
        gpu.gp0((100 << 16) | 103);
        gpu.gp0((102 << 16) | 100);
        assert_ne!(gpu.vram_pixel(102, 101) & 0x001f, 0);
        assert_eq!(gpu.vram_pixel(102, 100) & 0x001f, 0);
        assert_ne!(gpu.vram_pixel(100, 102) & 0x001f, 0);
        assert_ne!(gpu.vram_pixel(103, 100) & 0x001f, 0);
    }

    #[test]
    fn gouraud_line_interpolates_endpoint_colors() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x5000_00ff);
        gpu.gp0((220 << 16) | 200);
        gpu.gp0(0x0000_ff00);
        gpu.gp0((220 << 16) | 220);
        let pixel = gpu.vram_pixel(210, 220);
        assert_ne!(pixel & 0x001f, 0);
        assert_ne!(pixel & 0x03e0, 0);
    }

    #[test]
    fn gp1_internal_register_reads_return_drawing_environment() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0xe200_0000 | 0x000a_9421);
        gpu.gp0(0xe300_0000 | 17 | (23 << 10));
        gpu.gp0(0xe400_0000 | 511 | (255 << 10));
        gpu.gp0(0xe500_0000 | 0x07fb | (0x0007 << 11));

        gpu.gp1(0x1000_0002);
        assert_eq!(gpu.gpuread(), 0x000a_9421);
        gpu.gp1(0x1000_0003);
        assert_eq!(gpu.gpuread(), 17 | (23 << 10));
        gpu.gp1(0x1000_0004);
        assert_eq!(gpu.gpuread(), 511 | (255 << 10));
        gpu.gp1(0x1000_0005);
        assert_eq!(gpu.gpuread(), 0x07fb | (0x0007 << 11));
        gpu.gp1(0x1000_0007);
        assert_eq!(gpu.gpuread(), 2);
    }

    #[test]
    fn interlaced_480i_skips_active_field_unless_draw_mode_allows_it() {
        let mut gpu = Ps1Gpu::new();
        gpu.display_mode = 0x24;
        gpu.display_y = 0;
        gpu.frame = 0;

        gpu.write_render_pixel(10, 0, 0x001f, false);
        gpu.write_render_pixel(10, 1, 0x001f, false);
        assert_eq!(gpu.vram_pixel(10, 0), 0);
        assert_ne!(gpu.vram_pixel(10, 1), 0);

        gpu.frame = 1;
        gpu.write_render_pixel(11, 0, 0x03e0, false);
        gpu.write_render_pixel(11, 1, 0x03e0, false);
        assert_ne!(gpu.vram_pixel(11, 0), 0);
        assert_eq!(gpu.vram_pixel(11, 1), 0);

        gpu.draw_mode |= 1 << 10;
        gpu.write_render_pixel(12, 1, 0x7c00, false);
        assert_ne!(gpu.vram_pixel(12, 1), 0);

        gpu.draw_mode &= !(1 << 10);
        gpu.frame = 0;
        gpu.fill_rect(&[0x0200_00ff, 0, (4 << 16) | 16]);
        assert_eq!(gpu.vram_pixel(2, 0), 0);
        assert_ne!(gpu.vram_pixel(2, 1), 0);
        assert_eq!(gpu.vram_pixel(2, 2), 0);
        assert_ne!(gpu.vram_pixel(2, 3), 0);
    }

    #[test]
    fn display_snapshot_occurs_at_vblank_start_before_blank_period_vram_updates() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp1(0x0300_0000);
        gpu.vram[0] = 0x001f;

        let cycles_per_frame = gpu.cpu_cycles_per_frame();
        let scanlines = gpu.scanlines_per_frame();
        let vblank_phase = u64::from(gpu.display_v_end)
            .saturating_mul(cycles_per_frame)
            .div_ceil(scanlines);
        gpu.cycle_phase = vblank_phase - 2;
        gpu.tick_cpu_cycles(2);
        assert_eq!(gpu.frame(), 1);
        assert_eq!(&gpu.video().pixels()[..4], &[255, 0, 0, 255]);

        gpu.vram[0] = 0x03e0;
        let remaining = cycles_per_frame - gpu.cycle_phase;
        gpu.tick_cpu_cycles(remaining as u32);
        assert_eq!(gpu.frame(), 1);
        assert_eq!(&gpu.video().pixels()[..4], &[255, 0, 0, 255]);

        gpu.tick_cpu_cycles(vblank_phase as u32);
        assert_eq!(gpu.frame(), 2);
        assert_eq!(&gpu.video().pixels()[..4], &[0, 255, 0, 255]);
    }

    #[test]
    fn gpustat_field_and_display_line_bits_follow_interlace_timing() {
        let mut gpu = Ps1Gpu::new();
        gpu.display_y = 5;
        gpu.display_v_start = 16;
        gpu.display_v_end = 256;
        let visible_scanline = 20u64;
        gpu.cycle_phase = visible_scanline * gpu.cpu_cycles_per_frame() / gpu.scanlines_per_frame();

        let actual_scanline = gpu.timing_position().0;
        assert!(!gpu.in_vblank());
        let status = gpu.status();
        assert_ne!(status & (1 << 13), 0);
        assert_eq!((status >> 31) & 1, ((5 + actual_scanline) & 1) as u32);

        gpu.display_mode = 0x24;
        gpu.frame = 0;
        let status = gpu.status();
        assert_ne!(status & (1 << 13), 0);
        assert_eq!((status >> 31) & 1, 1);
        gpu.frame = 1;
        let status = gpu.status();
        assert_eq!(status & (1 << 13), 0);
        assert_eq!((status >> 31) & 1, 0);

        gpu.cycle_phase = 0;
        assert_eq!(gpu.status() & (1 << 31), 0);
    }

    #[test]
    fn gpustat_command_and_dma_readiness_tracks_partial_packets() {
        let mut gpu = Ps1Gpu::new();
        let initial = gpu.status();
        assert_ne!(initial & (1 << 26), 0);
        assert_ne!(initial & (1 << 28), 0);

        gpu.gp1(0x0400_0002);
        gpu.gp0(0x0200_00ff);
        let partial_fill = gpu.status();
        assert_eq!(partial_fill & (1 << 26), 0);
        assert_ne!(partial_fill & (1 << 28), 0);
        assert_ne!(partial_fill & (1 << 25), 0);
        gpu.gp0(0);
        assert_eq!(gpu.status() & (1 << 26), 0);
        gpu.gp0((1 << 16) | 16);
        let fill_busy = gpu.status();
        assert_eq!(fill_busy & (1 << 26), 0);
        assert_eq!(fill_busy & (1 << 28), 0);
        assert_eq!(gpu.pending_command_ticks, 57);
        gpu.tick_cpu_cycles(28);
        assert_eq!(gpu.status() & (1 << 28), 0);
        gpu.tick_cpu_cycles(1);
        let fill_done = gpu.status();
        assert_ne!(fill_done & (1 << 26), 0);
        assert_ne!(fill_done & (1 << 28), 0);

        gpu.gp0(0x2000_00ff);
        let partial_polygon = gpu.status();
        assert_eq!(partial_polygon & (1 << 26), 0);
        assert_eq!(partial_polygon & (1 << 28), 0);
        assert_eq!(partial_polygon & (1 << 25), 0);

        gpu.gp1(0x0400_0001);
        assert_ne!(gpu.status() & (1 << 25), 0);
        gpu.gp0(0);
        gpu.gp0(1);
        gpu.gp0(1 << 16);
        let polygon_busy = gpu.status();
        assert_eq!(polygon_busy & (1 << 26), 0);
        assert_eq!(polygon_busy & (1 << 28), 0);
        assert_eq!(gpu.pending_command_ticks, 46);
        gpu.tick_cpu_cycles(23);
        let polygon_done = gpu.status();
        assert_ne!(polygon_done & (1 << 26), 0);
        assert_ne!(polygon_done & (1 << 28), 0);
    }

    #[test]
    fn gpustat_transfer_phases_refresh_command_and_dma_readiness() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp1(0x0400_0002);
        gpu.gp0(0xa000_0000);
        gpu.gp0(0);
        gpu.gp0((1 << 16) | 2);
        let write_phase = gpu.status();
        assert_eq!(write_phase & (1 << 26), 0);
        assert_ne!(write_phase & (1 << 28), 0);
        assert_ne!(write_phase & (1 << 25), 0);
        gpu.gp0(0x5678_1234);
        let write_done = gpu.status();
        assert_ne!(write_done & (1 << 26), 0);
        assert_ne!(write_done & (1 << 28), 0);

        gpu.gp1(0x0400_0003);
        gpu.gp0(0xc000_0000);
        gpu.gp0(0);
        gpu.gp0((1 << 16) | 2);
        let read_phase = gpu.status();
        assert_eq!(read_phase & (1 << 26), 0);
        assert_ne!(read_phase & (1 << 27), 0);
        assert_eq!(read_phase & (1 << 28), 0);
        assert_ne!(read_phase & (1 << 25), 0);
        assert_eq!(gpu.gpuread(), 0x5678_1234);
        let read_done = gpu.status();
        assert_ne!(read_done & (1 << 26), 0);
        assert_eq!(read_done & (1 << 27), 0);
        assert_ne!(read_done & (1 << 28), 0);
    }

    #[test]
    fn gpustat_vram_readiness_tracks_active_transfer() {
        let mut gpu = Ps1Gpu::new();
        assert_eq!((gpu.status() >> 27) & 1, 0);
        gpu.gp0(0xc000_0000);
        gpu.gp0(0);
        gpu.gp0((1 << 16) | 1);
        assert_eq!((gpu.status() >> 27) & 1, 1);
        let _ = gpu.gpuread();
        assert_eq!((gpu.status() >> 27) & 1, 0);
    }

    #[test]
    fn display_mode_and_ranges_control_surface_dimensions() {
        let mut gpu = Ps1Gpu::new();
        let x1 = 0x0260u32;
        let x2 = x1 + 320 * 8;
        gpu.gp1(0x0600_0000 | x1 | (x2 << 12));
        gpu.gp1(0x0700_0000 | 0x10 | (0x100 << 10));
        gpu.gp1(0x0800_0001);
        gpu.gp1(0x0300_0000);
        gpu.tick_cpu_cycles(564_480);
        assert_eq!((gpu.video().width(), gpu.video().height()), (320, 240));

        gpu.gp1(0x0800_0002);
        gpu.tick_cpu_cycles(564_480);
        assert_eq!((gpu.video().width(), gpu.video().height()), (512, 240));
    }

    #[test]
    fn pal_display_mode_changes_frame_pacing() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp1(0x0800_0008);
        assert_eq!(gpu.frame_rate(), 50.0);
        assert_eq!(gpu.cpu_cycles_per_frame(), 677_376);
        let vblank_phase = u64::from(gpu.display_v_end)
            .saturating_mul(gpu.cpu_cycles_per_frame())
            .div_ceil(gpu.scanlines_per_frame());
        gpu.tick_cpu_cycles((vblank_phase - 1) as u32);
        assert_eq!(gpu.frame(), 0);
        gpu.tick_cpu_cycles(1);
        assert_eq!(gpu.frame(), 1);
        gpu.tick_cpu_cycles((gpu.cpu_cycles_per_frame() - vblank_phase) as u32);
        assert_eq!(gpu.frame(), 1);
        assert_ne!(gpu.status() & (1 << 20), 0);
    }

    #[test]
    fn twenty_four_bit_display_unpacks_packed_vram_bytes() {
        let mut gpu = Ps1Gpu::new();
        gpu.vram[0] = 0x140a;
        gpu.vram[1] = 0x281e;
        gpu.vram[2] = 0x3c32;
        gpu.gp1(0x0600_0000 | (32 << 12));
        gpu.gp1(0x0700_0000 | (1 << 10));
        gpu.gp1(0x0800_0011);
        gpu.gp1(0x0300_0000);
        gpu.tick_cpu_cycles(564_480);
        assert_eq!((gpu.video().width(), gpu.video().height()), (4, 1));
        assert_eq!(&gpu.video().pixels()[0..4], &[10, 20, 30, 255]);
        assert_eq!(&gpu.video().pixels()[4..8], &[40, 50, 60, 255]);
        assert_ne!(gpu.status() & (1 << 21), 0);
    }

    #[test]
    fn gpu_state_round_trip_preserves_fifo_vram_and_busy_timing() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x0200_00ff);
        gpu.gp0((4 << 16) | 3);
        gpu.gp0((2 << 16) | 2);
        assert_eq!(gpu.pending_command_ticks, 68);
        assert!(gpu.gp0(0x6800_ff00));
        assert!(gpu.gp0((40 << 16) | 40));
        assert_eq!(gpu.fifo.len(), 2);

        let mut out = StateWriter::new(crate::platform::PlatformId::PlayStation, 22);
        gpu.save(&mut out);
        let bytes = out.finish();
        gpu.reset();
        assert_eq!(gpu.pending_command_ticks, 0);
        assert!(gpu.fifo.is_empty());

        let mut input =
            StateReader::new(&bytes, crate::platform::PlatformId::PlayStation, 22).unwrap();
        gpu.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_ne!(gpu.vram_pixel(3, 4), 0);
        assert_eq!(gpu.pending_command_ticks, 68);
        assert_eq!(gpu.fifo.len(), 2);
        gpu.tick_cpu_cycles(34);
        assert!(gpu.fifo.is_empty());
        assert_ne!(gpu.vram_pixel(40, 40) & 0x03e0, 0);
    }
}
