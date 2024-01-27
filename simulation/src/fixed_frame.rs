pub struct FixedFrameGenerator {
    step: f32,
    last_time: f32,
    first_frame: bool,
}

impl FixedFrameGenerator {
    #[inline]
    pub fn new(time_step: f32) -> Self {
        Self {
            step: time_step,
            last_time: 0.0,
            first_frame: true,
        }
    }

    pub fn next(&mut self, current_time: f32) -> bool {
        if self.first_frame {
            self.first_frame = false;
            self.last_time = current_time;
            true
        } else {
            let delta_time = current_time - self.last_time;
            if delta_time >= self.step {
                self.last_time += self.step;
                true
            } else {
                false
            }
        }
    }
}
