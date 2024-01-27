use std::time::Instant;

pub struct FPSCounter {
    frame_count: usize,
    last_time: Instant,
    fps: u32,
}

impl Default for FPSCounter {
    fn default() -> Self {
        Self {
            frame_count: 0,
            last_time: Instant::now(),
            fps: 0,
        }
    }
}
impl FPSCounter {
    pub fn update(&mut self) {
        self.frame_count += 1;
        let now = Instant::now();
        let elapsed = now - self.last_time;
        if elapsed.as_secs_f32() > 1.0 {
            self.fps = (self.frame_count as f32 / elapsed.as_secs_f32()) as u32;
            self.frame_count = 0;
            self.last_time = now;
        }
    }

    pub fn fps(&self) -> u32 {
        self.fps
    }
}
