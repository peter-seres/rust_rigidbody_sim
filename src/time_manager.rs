use std::time;


pub struct TimeStep 
{
    last_time: time::Instant,
    delta_time: f32,
    last_render_time: time::Instant,
    render_dt: f32,
}

impl TimeStep {

    const MS_PER_FRAME: f32 = 16.7;

    pub fn new() -> TimeStep {
        TimeStep {
            last_time: time::Instant::now(),
            delta_time: 0.0,
            last_render_time: time::Instant::now(),
            render_dt: 0.0,
        }
    }

    pub fn delta(&mut self) -> f32 {
        let current_time = time::Instant::now();
        let delta = current_time.duration_since(self.last_time).as_millis() as f32;
        self.last_time = current_time;
        self.delta_time = delta;

        delta
    }

    pub fn should_render(&mut self) -> bool {
        let current_time = time::Instant::now();
        let delta_render = current_time.duration_since(self.last_render_time).as_millis() as f32;

        if delta_render >= TimeStep::MS_PER_FRAME {
            self.last_render_time = current_time;
            self.render_dt = delta_render;
            true
        } else {
            false
        }
    }
}
