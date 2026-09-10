use crate::kernel::VideoBuffer;
use crate::state::{StateReader, StateWriter};

const VRAM_WIDTH: usize = 1024;
const VRAM_HEIGHT: usize = 512;
const OUTPUT_WIDTH: u32 = 320;
const OUTPUT_HEIGHT: u32 = 240;

#[derive(Clone, Copy, Default)]
struct Transfer {
    x: u16,
    y: u16,
    width: u16,
    height: u16,
    index: u32,
    total: u32,
}

pub struct Ps1Gpu {
    vram: Vec<u16>,
    video: VideoBuffer,
    command: Vec<u32>,
    transfer_write: Option<Transfer>,
    transfer_read: Option<Transfer>,
    display_x: u16,
    display_y: u16,
    display_disabled: bool,
    dma_direction: u8,
    status: u32,
    frame: u64,
    cycle_phase: u64,
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
            transfer_write: None,
            transfer_read: None,
            display_x: 0,
            display_y: 0,
            display_disabled: true,
            dma_direction: 0,
            status: 0x1480_2000,
            frame: 0,
            cycle_phase: 0,
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
            0x02 | 0x60..=0x63 => 3,
            0x20..=0x23 => 4,
            0x28..=0x2b => 5,
            0x68..=0x6b => 2,
            0x80..=0x9f => 4,
            0xa0..=0xdf => 3,
            _ => 1,
        }
    }

    pub fn gp0(&mut self, value: u32) {
        if let Some(mut transfer) = self.transfer_write.take() {
            self.write_transfer_half(&mut transfer, value as u16);
            if transfer.index < transfer.total {
                self.write_transfer_half(&mut transfer, (value >> 16) as u16);
            }
            if transfer.index < transfer.total {
                self.transfer_write = Some(transfer);
            }
            return;
        }
        self.command.push(value);
        let expected = Self::command_words((self.command[0] >> 24) as u8);
        if self.command.len() >= expected {
            let command = std::mem::take(&mut self.command);
            self.execute_gp0(&command);
        }
    }
    fn execute_gp0(&mut self, command: &[u32]) {
        let opcode = (command[0] >> 24) as u8;
        match opcode {
            0x00 | 0x01 => {}
            0x1f => self.request_irq(),
            0x02 => self.fill_rect(command),
            0x20..=0x23 => self.flat_polygon(command, 3),
            0x28..=0x2b => self.flat_polygon(command, 4),
            0x60..=0x63 => self.variable_rect(command),
            0x68..=0x6b => self.dot(command),
            0x80..=0x9f => self.copy_vram(command),
            0xa0..=0xbf => self.begin_cpu_to_vram(command),
            0xc0..=0xdf => self.begin_vram_to_cpu(command),
            0xe1 => self.status = (self.status & !0x0000_07ff) | (command[0] & 0x7ff),
            0xe2..=0xe6 => {}
            _ => {}
        }
    }

    fn color15(value: u32) -> u16 {
        let r = (value & 0xff) >> 3;
        let g = ((value >> 8) & 0xff) >> 3;
        let b = ((value >> 16) & 0xff) >> 3;
        (r | (g << 5) | (b << 10)) as u16
    }

    fn xy(value: u32) -> (i32, i32) {
        let x = value as u16 as i16 as i32;
        let y = (value >> 16) as u16 as i16 as i32;
        (x, y)
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

    fn fill_rect(&mut self, command: &[u32]) {
        let color = Self::color15(command[0]);
        let (x, y) = Self::xy(command[1]);
        let (width, height) = Self::wh(command[2]);
        let width = i32::from((width + 0x0f) & !0x0f);
        for dy in 0..i32::from(height) {
            for dx in 0..width {
                self.write_pixel(x + dx, y + dy, color);
            }
        }
    }

    fn edge(a: (i32, i32), b: (i32, i32), p: (i32, i32)) -> i64 {
        i64::from(p.0 - a.0) * i64::from(b.1 - a.1) - i64::from(p.1 - a.1) * i64::from(b.0 - a.0)
    }
    fn flat_triangle(&mut self, points: [(i32, i32); 3], color: u16) {
        let min_x = points.iter().map(|p| p.0).min().unwrap_or(0).max(0);
        let max_x = points
            .iter()
            .map(|p| p.0)
            .max()
            .unwrap_or(0)
            .min((VRAM_WIDTH - 1) as i32);
        let min_y = points.iter().map(|p| p.1).min().unwrap_or(0).max(0);
        let max_y = points
            .iter()
            .map(|p| p.1)
            .max()
            .unwrap_or(0)
            .min((VRAM_HEIGHT - 1) as i32);
        let area = Self::edge(points[0], points[1], points[2]);
        if area == 0 {
            return;
        }
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let p = (x, y);
                let e0 = Self::edge(points[0], points[1], p);
                let e1 = Self::edge(points[1], points[2], p);
                let e2 = Self::edge(points[2], points[0], p);
                if (e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0) {
                    self.write_pixel(x, y, color);
                }
            }
        }
    }

    fn flat_polygon(&mut self, command: &[u32], vertices: usize) {
        let color = Self::color15(command[0]);
        let p0 = Self::xy(command[1]);
        let p1 = Self::xy(command[2]);
        let p2 = Self::xy(command[3]);
        self.flat_triangle([p0, p1, p2], color);
        if vertices == 4 {
            let p3 = Self::xy(command[4]);
            self.flat_triangle([p1, p2, p3], color);
        }
    }
    fn variable_rect(&mut self, command: &[u32]) {
        let color = Self::color15(command[0]);
        let (x, y) = Self::xy(command[1]);
        let (width, height) = Self::wh(command[2]);
        for dy in 0..i32::from(height) {
            for dx in 0..i32::from(width) {
                self.write_pixel(x + dx, y + dy, color);
            }
        }
    }

    fn dot(&mut self, command: &[u32]) {
        let color = Self::color15(command[0]);
        let (x, y) = Self::xy(command[1]);
        self.write_pixel(x, y, color);
    }

    fn copy_vram(&mut self, command: &[u32]) {
        let (sx, sy) = Self::xy(command[1]);
        let (dx, dy) = Self::xy(command[2]);
        let (width, height) = Self::wh(command[3]);
        let mut copy = Vec::with_capacity(usize::from(width) * usize::from(height));
        for y in 0..i32::from(height) {
            for x in 0..i32::from(width) {
                copy.push(self.vram[Self::vram_index(sx + x, sy + y)]);
            }
        }
        let mut index = 0;
        for y in 0..i32::from(height) {
            for x in 0..i32::from(width) {
                self.write_pixel(dx + x, dy + y, copy[index]);
                index += 1;
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
        self.write_pixel(x as i32, y as i32, value);
        transfer.index += 1;
    }
    pub fn gpuread(&mut self) -> u32 {
        let Some(mut transfer) = self.transfer_read.take() else {
            return 0;
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
                self.transfer_write = None;
                self.transfer_read = None;
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
            0x08 => self.set_display_mode(value),
            _ => {}
        }
        self.refresh_status();
    }
    fn gpu_reset(&mut self) {
        self.command.clear();
        self.transfer_write = None;
        self.transfer_read = None;
        self.display_x = 0;
        self.display_y = 0;
        self.display_disabled = true;
        self.dma_direction = 0;
        self.status = 0x1480_2000;
        self.irq = false;
    }

    fn set_display_mode(&mut self, value: u32) {
        self.status &= !0x007f_4000;
        self.status |= (value & 0x3f) << 17;
        if value & 0x40 != 0 {
            self.status |= 1 << 16;
        }
    }

    fn refresh_status(&mut self) {
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
        self.status |= (1 << 26) | (1 << 27) | (1 << 28);
    }

    pub fn status(&mut self) -> u32 {
        self.refresh_status();
        self.status
    }

    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        const CPU_CYCLES_PER_FRAME: u64 = 564_480;
        self.cycle_phase += u64::from(cycles);
        while self.cycle_phase >= CPU_CYCLES_PER_FRAME {
            self.cycle_phase -= CPU_CYCLES_PER_FRAME;
            self.render_display();
            self.frame = self.frame.wrapping_add(1);
        }
    }

    fn render_display(&mut self) {
        if self.display_disabled {
            self.video.clear([0, 0, 0, 255]);
            return;
        }
        let display_x = usize::from(self.display_x);
        let display_y = usize::from(self.display_y);
        for y in 0..OUTPUT_HEIGHT as usize {
            for x in 0..OUTPUT_WIDTH as usize {
                let value = self.vram
                    [((display_y + y) % VRAM_HEIGHT) * VRAM_WIDTH + ((display_x + x) % VRAM_WIDTH)];
                let r = ((value & 0x1f) * 255 / 31) as u8;
                let g = (((value >> 5) & 0x1f) * 255 / 31) as u8;
                let b = (((value >> 10) & 0x1f) * 255 / 31) as u8;
                let offset = (y * OUTPUT_WIDTH as usize + x) * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&[r, g, b, 255]);
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
        Self::save_transfer(out, self.transfer_write);
        Self::save_transfer(out, self.transfer_read);
        out.u16(self.display_x);
        out.u16(self.display_y);
        out.u8(self.display_disabled as u8);
        out.u8(self.dma_direction);
        out.u32(self.status);
        out.u64(self.frame);
        out.u64(self.cycle_phase);
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
        self.transfer_write = Self::load_transfer(input)?;
        self.transfer_read = Self::load_transfer(input)?;
        self.display_x = input.u16()? & 0x03ff;
        self.display_y = input.u16()? & 0x01ff;
        self.display_disabled = input.u8()? != 0;
        self.dma_direction = input.u8()? & 3;
        self.status = input.u32()?;
        self.frame = input.u64()?;
        self.cycle_phase = input.u64()? % 564_480;
        self.irq = input.u8()? != 0;
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
    fn gpu_state_round_trip_preserves_vram() {
        let mut gpu = Ps1Gpu::new();
        gpu.gp0(0x0200_00ff);
        gpu.gp0((4 << 16) | 3);
        gpu.gp0((2 << 16) | 2);
        let mut out = StateWriter::new(crate::platform::PlatformId::PlayStation, 17);
        gpu.save(&mut out);
        let bytes = out.finish();
        gpu.reset();
        let mut input =
            StateReader::new(&bytes, crate::platform::PlatformId::PlayStation, 17).unwrap();
        gpu.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_ne!(gpu.vram_pixel(3, 4), 0);
    }
}
