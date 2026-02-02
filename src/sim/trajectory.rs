use raylib::prelude::*;

#[derive(Copy, Clone, Debug)]
pub struct LinearTrajectory {
    a: Vector3,
    b: Vector3,
    duration: f32,
    t: f32,
    alpha: f32,
    finished: bool,
}

impl Default for LinearTrajectory {
    fn default() -> Self {
        Self {
            a: Vector3::zero(),
            b: Vector3::zero(),
            duration: 1.0,
            t: 0.0,
            alpha: 0.0,
            finished: true,
        }
    }
}

impl LinearTrajectory {
    pub fn reset(&mut self, from: Vector3, to: Vector3, duration_sec: f32) {
        self.a = from;
        self.b = to;
        self.duration = if duration_sec > 1e-6 { duration_sec } else { 1e-6 };
        self.t = 0.0;
        self.alpha = 0.0;
        self.finished = false;
    }

    pub fn update(&mut self, dt: f32) {
        if self.finished {
            return;
        }
        self.t += dt;
        self.alpha = (self.t / self.duration).clamp(0.0, 1.0);
        if self.alpha >= 1.0 {
            self.finished = true;
        }
    }

    pub fn position(&self) -> Vector3 {
        Vector3 {
            x: self.a.x + (self.b.x - self.a.x) * self.alpha,
            y: self.a.y + (self.b.y - self.a.y) * self.alpha,
            z: self.a.z + (self.b.z - self.a.z) * self.alpha,
        }
    }

    pub fn finished(&self) -> bool {
        self.finished
    }
}
