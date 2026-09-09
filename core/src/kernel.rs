use std::cmp::Reverse;
use std::collections::BinaryHeap;

pub const MAX_PLAYERS: usize = 4;

#[derive(Clone)]
pub struct InputState {
    pub buttons: [u64; MAX_PLAYERS],
}

impl Default for InputState {
    fn default() -> Self {
        Self {
            buttons: [0; MAX_PLAYERS],
        }
    }
}

pub struct VideoBuffer {
    width: u32,
    height: u32,
    pixels: Vec<u8>,
}

impl VideoBuffer {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            pixels: vec![0; (width * height * 4) as usize],
        }
    }
    pub fn resize(&mut self, width: u32, height: u32) {
        self.width = width;
        self.height = height;
        self.pixels.resize((width * height * 4) as usize, 0);
    }
    pub fn clear(&mut self, rgba: [u8; 4]) {
        for pixel in self.pixels.as_chunks_mut::<4>().0 {
            *pixel = rgba;
        }
    }
    pub fn width(&self) -> u32 {
        self.width
    }
    pub fn height(&self) -> u32 {
        self.height
    }
    pub fn pixels(&self) -> &[u8] {
        &self.pixels
    }
    pub fn pixels_mut(&mut self) -> &mut [u8] {
        &mut self.pixels
    }
}

pub struct AudioBuffer {
    sample_rate: u32,
    channels: u32,
    samples: Vec<f32>,
}

impl AudioBuffer {
    pub fn new(sample_rate: u32, channels: u32) -> Self {
        Self {
            sample_rate,
            channels,
            samples: Vec::with_capacity(4096),
        }
    }
    pub fn begin_frame(&mut self) {
        self.samples.clear();
    }
    pub fn push_stereo(&mut self, left: f32, right: f32) {
        self.samples.push(left);
        self.samples.push(right);
    }
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }
    pub fn channels(&self) -> u32 {
        self.channels
    }
    pub fn samples(&self) -> &[f32] {
        &self.samples
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ScheduledEvent {
    pub at: u64,
    pub sequence: u64,
    pub device: u16,
    pub code: u16,
}

pub struct Scheduler {
    now: u64,
    sequence: u64,
    queue: BinaryHeap<Reverse<ScheduledEvent>>,
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Scheduler {
    pub fn new() -> Self {
        Self {
            now: 0,
            sequence: 0,
            queue: BinaryHeap::new(),
        }
    }
    pub fn now(&self) -> u64 {
        self.now
    }
    pub fn schedule_at(&mut self, at: u64, device: u16, code: u16) {
        let event = ScheduledEvent {
            at,
            sequence: self.sequence,
            device,
            code,
        };
        self.sequence = self.sequence.wrapping_add(1);
        self.queue.push(Reverse(event));
    }
    pub fn advance_to(&mut self, target: u64) -> Vec<ScheduledEvent> {
        let mut ready = Vec::new();
        while self.queue.peek().is_some_and(|event| event.0.at <= target) {
            ready.push(self.queue.pop().unwrap().0);
        }
        self.now = target;
        ready
    }
}
