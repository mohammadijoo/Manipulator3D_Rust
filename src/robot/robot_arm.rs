use raylib::prelude::*;

#[derive(Copy, Clone, Debug)]
pub struct LinkParams {
    pub length_m: f32,
    pub mass_kg: f32,
    pub inertia_cm: f32,
    pub inertia_joint: f32,
}

impl Default for LinkParams {
    fn default() -> Self {
        Self {
            length_m: 2.5,
            mass_kg: 1.0,
            inertia_cm: 0.0,
            inertia_joint: 0.0,
        }
    }
}

impl LinkParams {
    pub fn recompute_inertia(&mut self) {
        // Uniform rod approximations (axis perpendicular to rod)
        self.inertia_cm = (1.0 / 12.0) * self.mass_kg * self.length_m * self.length_m;
        self.inertia_joint = (1.0 / 3.0) * self.mass_kg * self.length_m * self.length_m;
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct JointAngles {
    pub q0_yaw: f32,
    pub q1_pitch: f32,
    pub q2_pitch: f32,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct FKResult {
    pub base: Vector3,
    pub joint1: Vector3,
    pub joint2: Vector3,
    pub ee: Vector3,
}

#[derive(Clone, Debug)]
pub struct IKResult {
    pub reachable: bool,
    pub q: JointAngles,
    pub message: String,
}

impl Default for IKResult {
    fn default() -> Self {
        Self {
            reachable: false,
            q: JointAngles::default(),
            message: String::new(),
        }
    }
}

pub struct RobotArm {
    link1: LinkParams,
    link2: LinkParams,
}

impl RobotArm {
    pub fn new(mut l1: LinkParams, mut l2: LinkParams) -> Self {
        l1.recompute_inertia();
        l2.recompute_inertia();
        Self { link1: l1, link2: l2 }
    }

    pub fn link1(&self) -> &LinkParams { &self.link1 }
    pub fn link2(&self) -> &LinkParams { &self.link2 }

    pub fn l1(&self) -> f32 { self.link1.length_m }
    pub fn l2(&self) -> f32 { self.link2.length_m }

    pub fn max_reach(&self) -> f32 { self.link1.length_m + self.link2.length_m }
    pub fn min_reach(&self) -> f32 { (self.link1.length_m - self.link2.length_m).abs() }

    pub fn solve_ik(&self, target: Vector3, elbow_up: bool) -> IKResult {
        let mut out = IKResult::default();

        if target.z < 0.0 {
            out.reachable = false;
            out.message = "Invalid target: z must be >= 0".to_string();
            return out;
        }

        let x = target.x as f64;
        let y = target.y as f64;
        let z = target.z as f64;

        let d = (x * x + y * y + z * z).sqrt();
        let rmin = self.min_reach() as f64;
        let rmax = self.max_reach() as f64;

        if d < rmin - 1e-9 || d > rmax + 1e-9 {
            out.reachable = false;
            out.message = format!("Target radius |p|={} is outside [{}, {}]", d, rmin, rmax);
            return out;
        }

        // Base yaw in XY plane
        let mut q0 = 0.0_f64;
        if x.abs() > 1e-12 || y.abs() > 1e-12 {
            q0 = y.atan2(x);
        }

        // Reduce to planar (r,z) where r = sqrt(x^2+y^2)
        let r = (x * x + y * y).sqrt();
        let l1 = self.l1() as f64;
        let l2 = self.l2() as f64;

        // Law of cosines for elbow angle
        let mut c2 = (r * r + z * z - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
        if c2 < -1.0 { c2 = -1.0; }
        if c2 >  1.0 { c2 =  1.0; }

        let mut q2 = c2.acos();
        if !elbow_up {
            q2 = -q2;
        }

        // Shoulder angle
        let s2 = q2.sin();
        let k1 = l1 + l2 * q2.cos();
        let k2 = l2 * s2;

        let q1 = z.atan2(r) - k2.atan2(k1);

        out.reachable = true;
        out.q.q0_yaw = q0 as f32;
        out.q.q1_pitch = q1 as f32;
        out.q.q2_pitch = q2 as f32;
        out.message = "OK".to_string();
        out
    }

    pub fn forward_kinematics(&self, q: JointAngles) -> FKResult {
        let mut fk = FKResult::default();
        fk.base = Vector3::zero();
        fk.joint1 = fk.base;

        let l1 = self.l1();
        let l2 = self.l2();

        let cy = q.q0_yaw.cos();
        let sy = q.q0_yaw.sin();
        let u = Vector3 { x: cy, y: sy, z: 0.0 };
        let k = Vector3 { x: 0.0, y: 0.0, z: 1.0 };

        // Elbow
        let p1 = Vector3 {
            x: u.x * (l1 * q.q1_pitch.cos()) + k.x * (l1 * q.q1_pitch.sin()),
            y: u.y * (l1 * q.q1_pitch.cos()) + k.y * (l1 * q.q1_pitch.sin()),
            z: u.z * (l1 * q.q1_pitch.cos()) + k.z * (l1 * q.q1_pitch.sin()),
        };

        // EE
        let a = q.q1_pitch + q.q2_pitch;
        let p2 = Vector3 {
            x: p1.x + u.x * (l2 * a.cos()) + k.x * (l2 * a.sin()),
            y: p1.y + u.y * (l2 * a.cos()) + k.y * (l2 * a.sin()),
            z: p1.z + u.z * (l2 * a.cos()) + k.z * (l2 * a.sin()),
        };

        fk.joint2 = p1;
        fk.ee = p2;
        fk
    }
}
