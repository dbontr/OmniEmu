use crate::input::{
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_Y, DOWN, L1, L2, LEFT, R1, R2, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const FPS: f64 = 60.0;
const PLAYER_SIZE: i32 = 8;
const BALL_SIZE: i32 = 6;
const STATE_VERSION: u32 = 1;

#[derive(Debug, Clone, Copy)]
struct CardWiring {
    ball: bool,
    gate: bool,
    english: bool,
    wall: bool,
    player1: bool,
    player2: bool,
}

impl CardWiring {
    const fn for_card(card: u8) -> Self {
        match card {
            1 => Self {
                ball: true,
                gate: true,
                english: true,
                wall: true,
                player1: true,
                player2: true,
            },
            2 => Self {
                ball: false,
                gate: false,
                english: false,
                wall: false,
                player1: true,
                player2: true,
            },
            3 => Self {
                ball: true,
                gate: true,
                english: true,
                wall: false,
                player1: true,
                player2: true,
            },
            4 => Self {
                ball: false,
                gate: false,
                english: false,
                wall: true,
                player1: true,
                player2: true,
            },
            5 => Self {
                ball: true,
                gate: true,
                english: true,
                wall: false,
                player1: true,
                player2: true,
            },
            6 => Self {
                ball: false,
                gate: false,
                english: false,
                wall: false,
                player1: false,
                player2: true,
            },
            _ => Self::for_card(1),
        }
    }
}

pub struct OdysseyMachine {
    video: VideoBuffer,
    audio: AudioBuffer,
    card: u8,
    player_x: [f32; 2],
    player_y: [f32; 2],
    ball_x: f32,
    ball_y: f32,
    ball_vx: f32,
    ball_vy: f32,
    ball_visible: bool,
    wall_x: f32,
    speed: f32,
    previous_buttons: [u64; 2],
}

impl Default for OdysseyMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl OdysseyMachine {
    pub fn new() -> Self {
        let mut machine = Self {
            video: VideoBuffer::new(WIDTH, HEIGHT),
            audio: AudioBuffer::new(48_000, 2),
            card: 1,
            player_x: [72.0, 240.0],
            player_y: [112.0, 112.0],
            ball_x: 157.0,
            ball_y: 117.0,
            ball_vx: 1.9,
            ball_vy: 0.0,
            ball_visible: true,
            wall_x: 159.0,
            speed: 1.9,
            previous_buttons: [0; 2],
        };
        machine.render();
        machine
    }

    pub fn card(&self) -> u8 {
        self.card
    }

    fn wiring(&self) -> CardWiring {
        CardWiring::for_card(self.card)
    }

    fn selected_edge(&self, input: &InputState, player: usize, button: u64) -> bool {
        input.buttons[player] & button != 0 && self.previous_buttons[player] & button == 0
    }

    fn normalized_axis(value: i16) -> f32 {
        f32::from(value) / 32767.0
    }

    fn update_controls(&mut self, input: &InputState) {
        if self.selected_edge(input, 0, SELECT) {
            self.card = if self.card == 6 { 1 } else { self.card + 1 };
            self.ball_visible = self.wiring().ball;
        }
        let wiring = self.wiring();
        for player in 0..2 {
            let enabled = if player == 0 {
                wiring.player1
            } else {
                wiring.player2
            };
            if !enabled {
                continue;
            }
            let buttons = input.buttons[player];
            let digital_x =
                f32::from((buttons & RIGHT != 0) as u8) - f32::from((buttons & LEFT != 0) as u8);
            let digital_y =
                f32::from((buttons & DOWN != 0) as u8) - f32::from((buttons & UP != 0) as u8);
            let analog_x = Self::normalized_axis(input.axes[player][AXIS_LEFT_X]);
            let analog_y = Self::normalized_axis(input.axes[player][AXIS_LEFT_Y]);
            let x = if analog_x.abs() > 0.08 {
                analog_x
            } else {
                digital_x
            };
            let y = if analog_y.abs() > 0.08 {
                analog_y
            } else {
                digital_y
            };
            self.player_x[player] = (self.player_x[player] + x * 2.5)
                .clamp(4.0, WIDTH as f32 - PLAYER_SIZE as f32 - 4.0);
            self.player_y[player] = (self.player_y[player] + y * 2.5)
                .clamp(4.0, HEIGHT as f32 - PLAYER_SIZE as f32 - 4.0);
        }

        if input.buttons[0] & L1 != 0 {
            self.wall_x = (self.wall_x - 0.7).max(20.0);
        }
        if input.buttons[0] & R1 != 0 {
            self.wall_x = (self.wall_x + 0.7).min(WIDTH as f32 - 20.0);
        }
        if input.buttons[0] & L2 != 0 {
            self.speed = (self.speed - 0.02).max(0.5);
        }
        if input.buttons[0] & R2 != 0 {
            self.speed = (self.speed + 0.02).min(5.0);
        }

        if wiring.ball {
            if self.selected_edge(input, 0, START) {
                self.reset_ball(1.0);
            }
            if self.selected_edge(input, 1, START) {
                self.reset_ball(-1.0);
            }
        }
    }

    fn reset_ball(&mut self, direction: f32) {
        self.ball_x = WIDTH as f32 * 0.5 - BALL_SIZE as f32 * 0.5;
        self.ball_y = HEIGHT as f32 * 0.5 - BALL_SIZE as f32 * 0.5;
        self.ball_vx = self.speed * direction;
        self.ball_vy = 0.0;
        self.ball_visible = true;
    }

    fn overlap(ball_x: f32, ball_y: f32, player_x: f32, player_y: f32) -> bool {
        ball_x < player_x + PLAYER_SIZE as f32
            && ball_x + BALL_SIZE as f32 > player_x
            && ball_y < player_y + PLAYER_SIZE as f32
            && ball_y + BALL_SIZE as f32 > player_y
    }

    fn update_ball(&mut self, input: &InputState) {
        let wiring = self.wiring();
        if !wiring.ball || !self.ball_visible {
            return;
        }
        self.ball_vx = self.speed.copysign(self.ball_vx);
        if wiring.english {
            let player = usize::from(self.ball_vx < 0.0);
            let english = Self::normalized_axis(input.axes[player][AXIS_RIGHT_Y]);
            self.ball_vy = (self.ball_vy + english * 0.08).clamp(-3.5, 3.5);
        }
        self.ball_x += self.ball_vx;
        self.ball_y += self.ball_vy;
        if self.ball_y < 0.0 || self.ball_y > HEIGHT as f32 - BALL_SIZE as f32 {
            self.ball_visible = false;
            return;
        }
        if self.ball_x < 0.0 || self.ball_x > WIDTH as f32 - BALL_SIZE as f32 {
            self.ball_visible = false;
            return;
        }
        if wiring.gate {
            for player in 0..2 {
                let enabled = if player == 0 {
                    wiring.player1
                } else {
                    wiring.player2
                };
                if enabled
                    && Self::overlap(
                        self.ball_x,
                        self.ball_y,
                        self.player_x[player],
                        self.player_y[player],
                    )
                {
                    self.ball_vx = -self.ball_vx;
                    self.ball_x += self.ball_vx.signum() * 2.0;
                }
            }
        }
    }

    fn rect(&mut self, x: i32, y: i32, width: i32, height: i32) {
        let x0 = x.clamp(0, WIDTH as i32);
        let y0 = y.clamp(0, HEIGHT as i32);
        let x1 = (x + width).clamp(0, WIDTH as i32);
        let y1 = (y + height).clamp(0, HEIGHT as i32);
        let pixels = self.video.pixels_mut();
        for py in y0..y1 {
            for px in x0..x1 {
                let offset = ((py as u32 * WIDTH + px as u32) * 4) as usize;
                pixels[offset..offset + 4].copy_from_slice(&[238, 238, 232, 255]);
            }
        }
    }

    fn render(&mut self) {
        self.video.clear([36, 36, 34, 255]);
        let wiring = self.wiring();
        if wiring.wall {
            self.rect(self.wall_x as i32, 0, 3, HEIGHT as i32);
        }
        if wiring.player1 {
            self.rect(
                self.player_x[0] as i32,
                self.player_y[0] as i32,
                PLAYER_SIZE,
                PLAYER_SIZE,
            );
        }
        if wiring.player2 {
            self.rect(
                self.player_x[1] as i32,
                self.player_y[1] as i32,
                PLAYER_SIZE,
                PLAYER_SIZE,
            );
        }
        if wiring.ball && self.ball_visible {
            self.rect(self.ball_x as i32, self.ball_y as i32, BALL_SIZE, BALL_SIZE);
        }
    }
}

impl Machine for OdysseyMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Odyssey
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.update_controls(input);
        self.update_ball(input);
        self.render();
        self.audio.begin_frame();
        self.previous_buttons[0] = input.buttons[0];
        self.previous_buttons[1] = input.buttons[1];
    }

    fn frame_rate(&self) -> f64 {
        FPS
    }

    fn video(&self) -> &VideoBuffer {
        &self.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Odyssey, STATE_VERSION);
        out.u8(self.card);
        for value in self.player_x.into_iter().chain(self.player_y) {
            out.f32(value);
        }
        for value in [
            self.ball_x,
            self.ball_y,
            self.ball_vx,
            self.ball_vy,
            self.wall_x,
            self.speed,
        ] {
            out.f32(value);
        }
        out.u8(u8::from(self.ball_visible));
        out.u64(self.previous_buttons[0]);
        out.u64(self.previous_buttons[1]);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Odyssey, STATE_VERSION)?;
        self.card = input.u8()?;
        if !(1..=6).contains(&self.card) {
            return Err("Odyssey state contains an invalid game card".into());
        }
        for value in &mut self.player_x {
            *value = input.f32()?;
        }
        for value in &mut self.player_y {
            *value = input.f32()?;
        }
        self.ball_x = input.f32()?;
        self.ball_y = input.f32()?;
        self.ball_vx = input.f32()?;
        self.ball_vy = input.f32()?;
        self.wall_x = input.f32()?;
        self.speed = input.f32()?;
        self.ball_visible = input.u8()? != 0;
        self.previous_buttons[0] = input.u64()?;
        self.previous_buttons[1] = input.u64()?;
        input.finish()?;
        self.render();
        self.audio.begin_frame();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn odyssey_card_one_exposes_discrete_spots_and_is_silent() {
        let mut machine = OdysseyMachine::new();
        assert_eq!(machine.card(), 1);
        machine.run_frame(&InputState::default());
        assert!(machine.video().pixels().iter().any(|value| *value > 200));
        assert!(machine.audio().samples().is_empty());
    }

    #[test]
    fn game_card_switch_rewires_visible_generators() {
        let mut machine = OdysseyMachine::new();
        let mut input = InputState::default();
        input.buttons[0] = SELECT;
        machine.run_frame(&input);
        assert_eq!(machine.card(), 2);
        assert!(!machine.ball_visible);
        input.buttons[0] = 0;
        machine.run_frame(&input);
        input.buttons[0] = SELECT;
        machine.run_frame(&input);
        assert_eq!(machine.card(), 3);
        assert!(machine.ball_visible);
    }

    #[test]
    fn state_round_trip_preserves_wiring_and_spot_positions() {
        let mut machine = OdysseyMachine::new();
        let mut input = InputState::default();
        input.buttons[0] = RIGHT | DOWN;
        for _ in 0..5 {
            machine.run_frame(&input);
        }
        let saved = machine.save_state().unwrap();
        let pixels = machine.video().pixels().to_vec();
        machine.reset();
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.save_state().unwrap(), saved);
        assert_eq!(machine.video().pixels(), pixels);
    }
}
