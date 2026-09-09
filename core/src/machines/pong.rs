use crate::input::{DOWN, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const PADDLE_W: i32 = 5;
const PADDLE_H: i32 = 38;
const BALL: i32 = 5;
const FPS: f32 = 60.0;

pub struct PongMachine {
    video: VideoBuffer,
    audio: AudioBuffer,
    left_y: f32,
    right_y: f32,
    ball_x: f32,
    ball_y: f32,
    ball_vx: f32,
    ball_vy: f32,
    score_left: u8,
    score_right: u8,
    tone_frames: u16,
    tone_phase: f32,
}
impl Default for PongMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl PongMachine {
    pub fn new() -> Self {
        let mut machine = Self {
            video: VideoBuffer::new(WIDTH, HEIGHT),
            audio: AudioBuffer::new(48_000, 2),
            left_y: 101.0,
            right_y: 101.0,
            ball_x: 157.0,
            ball_y: 117.0,
            ball_vx: 2.4,
            ball_vy: 1.45,
            score_left: 0,
            score_right: 0,
            tone_frames: 0,
            tone_phase: 0.0,
        };
        machine.render();
        machine
    }

    fn serve(&mut self, direction: f32) {
        self.ball_x = 157.0;
        self.ball_y = 117.0;
        self.ball_vx = 2.4 * direction;
        self.ball_vy = if (self.score_left + self.score_right) & 1 == 0 {
            1.45
        } else {
            -1.45
        };
    }
    fn update(&mut self, input: &InputState) {
        let speed = 3.2;
        if input.buttons[0] & UP != 0 {
            self.left_y -= speed;
        }
        if input.buttons[0] & DOWN != 0 {
            self.left_y += speed;
        }
        if input.buttons[1] & UP != 0 {
            self.right_y -= speed;
        }
        if input.buttons[1] & DOWN != 0 {
            self.right_y += speed;
        }
        self.left_y = self
            .left_y
            .clamp(4.0, (HEIGHT as i32 - PADDLE_H - 4) as f32);
        self.right_y = self
            .right_y
            .clamp(4.0, (HEIGHT as i32 - PADDLE_H - 4) as f32);

        self.ball_x += self.ball_vx;
        self.ball_y += self.ball_vy;
        if self.ball_y <= 3.0 || self.ball_y >= (HEIGHT as i32 - BALL - 3) as f32 {
            self.ball_y = self.ball_y.clamp(3.0, (HEIGHT as i32 - BALL - 3) as f32);
            self.ball_vy = -self.ball_vy;
            self.tone_frames = 3;
        }

        self.collide_paddle(14.0, self.left_y, 1.0);
        self.collide_paddle((WIDTH as i32 - 19) as f32, self.right_y, -1.0);
        if self.ball_x < -BALL as f32 {
            self.score_right = self.score_right.wrapping_add(1);
            self.serve(1.0);
        } else if self.ball_x > WIDTH as f32 {
            self.score_left = self.score_left.wrapping_add(1);
            self.serve(-1.0);
        }
    }
    fn collide_paddle(&mut self, paddle_x: f32, paddle_y: f32, direction: f32) {
        let ball_left = self.ball_x;
        let ball_right = self.ball_x + BALL as f32;
        let ball_top = self.ball_y;
        let ball_bottom = self.ball_y + BALL as f32;
        let hit_x = ball_left <= paddle_x + PADDLE_W as f32 && ball_right >= paddle_x;
        if hit_x
            && ball_bottom >= paddle_y
            && ball_top <= paddle_y + PADDLE_H as f32
            && self.ball_vx.signum() != direction
        {
            let relative = ((self.ball_y + BALL as f32 * 0.5) - (paddle_y + PADDLE_H as f32 * 0.5))
                / (PADDLE_H as f32 * 0.5);
            self.ball_vx = (self.ball_vx.abs() * 1.025).min(5.5) * direction;
            self.ball_vy = (self.ball_vy + relative * 0.75).clamp(-4.8, 4.8);
            self.ball_x = if direction > 0.0 {
                paddle_x + PADDLE_W as f32 + 0.5
            } else {
                paddle_x - BALL as f32 - 0.5
            };
            self.tone_frames = 5;
        }
    }

    fn render(&mut self) {
        self.video.clear([5, 8, 6, 255]);
        self.rect(
            14,
            self.left_y as i32,
            PADDLE_W,
            PADDLE_H,
            [222, 235, 224, 255],
        );
        self.rect(
            WIDTH as i32 - 19,
            self.right_y as i32,
            PADDLE_W,
            PADDLE_H,
            [222, 235, 224, 255],
        );
        self.rect(
            self.ball_x as i32,
            self.ball_y as i32,
            BALL,
            BALL,
            [131, 191, 106, 255],
        );
    }
    fn rect(&mut self, x: i32, y: i32, width: i32, height: i32, rgba: [u8; 4]) {
        let x0 = x.max(0).min(WIDTH as i32);
        let y0 = y.max(0).min(HEIGHT as i32);
        let x1 = (x + width).max(0).min(WIDTH as i32);
        let y1 = (y + height).max(0).min(HEIGHT as i32);
        let pixels = self.video.pixels_mut();
        for py in y0..y1 {
            for px in x0..x1 {
                let offset = ((py as u32 * WIDTH + px as u32) * 4) as usize;
                pixels[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    fn synth_audio(&mut self) {
        self.audio.begin_frame();
        let frames = (self.audio.sample_rate() as f32 / FPS).round() as usize;
        for _ in 0..frames {
            let sample = if self.tone_frames > 0 {
                if self.tone_phase < 0.5 {
                    0.12
                } else {
                    -0.12
                }
            } else {
                0.0
            };
            self.audio.push_stereo(sample, sample);
            self.tone_phase = (self.tone_phase + 880.0 / self.audio.sample_rate() as f32) % 1.0;
        }
        if self.tone_frames > 0 {
            self.tone_frames -= 1;
        }
    }
}
impl Machine for PongMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::HomePong
    }
    fn reset(&mut self) {
        self.left_y = 101.0;
        self.right_y = 101.0;
        self.score_left = 0;
        self.score_right = 0;
        self.tone_frames = 0;
        self.tone_phase = 0.0;
        self.serve(1.0);
        self.render();
        self.audio.begin_frame();
    }
    fn run_frame(&mut self, input: &InputState) {
        self.update(input);
        self.render();
        self.synth_audio();
    }
    fn frame_rate(&self) -> f64 {
        60.0
    }
    fn video(&self) -> &VideoBuffer {
        &self.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }
    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::HomePong, 1);
        for value in [
            self.left_y,
            self.right_y,
            self.ball_x,
            self.ball_y,
            self.ball_vx,
            self.ball_vy,
            self.tone_phase,
        ] {
            out.f32(value);
        }
        out.u8(self.score_left);
        out.u8(self.score_right);
        out.u16(self.tone_frames);
        Ok(out.finish())
    }
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::HomePong, 1)?;
        self.left_y = input.f32()?;
        self.right_y = input.f32()?;
        self.ball_x = input.f32()?;
        self.ball_y = input.f32()?;
        self.ball_vx = input.f32()?;
        self.ball_vy = input.f32()?;
        self.tone_phase = input.f32()?;
        self.score_left = input.u8()?;
        self.score_right = input.u8()?;
        self.tone_frames = input.u16()?;
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
    fn pong_runs_and_round_trips_state() {
        let mut machine = PongMachine::new();
        let mut input = InputState::default();
        input.buttons[0] = DOWN;
        for _ in 0..10 {
            machine.run_frame(&input);
        }
        let state = machine.save_state().unwrap();
        let pixels = machine.video().pixels().to_vec();
        machine.reset();
        machine.load_state(&state).unwrap();
        assert_eq!(machine.save_state().unwrap(), state);
        assert_eq!(machine.video().pixels(), pixels);
        assert_eq!(machine.audio().channels(), 2);
    }
}
