pub struct FixedFrames {
    step: f32,
    last_time: f32,
    first_frame: bool,
}

impl FixedFrames {
    #[inline]
    pub fn new(time_step: f32) -> Self {
        Self {
            step: time_step,
            last_time: 0.0,
            first_frame: true,
        }
    }

    pub fn iter(&mut self, current_time: f32, max_frame_count: usize) -> FixedFramesIterMut {
        FixedFramesIterMut {
            frames: self,
            current_time,
            frame_index: 0,
            max_frame_count,
        }
    }
}

pub struct FixedFramesIterMut<'a> {
    frames: &'a mut FixedFrames,
    current_time: f32,
    max_frame_count: usize,
    frame_index: usize,
}

impl Iterator for FixedFramesIterMut<'_> {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        if self.frame_index >= self.max_frame_count {
            return None;
        }
        self.frame_index += 1;
        if self.frames.first_frame {
            self.frames.first_frame = false;
            self.frames.last_time = self.current_time;
            Some(self.current_time)
        } else {
            let delta_time = self.current_time - self.frames.last_time;
            if delta_time >= self.frames.step {
                self.frames.last_time += self.frames.step;
                Some(self.frames.last_time)
            } else {
                None
            }
        }
    }
}
