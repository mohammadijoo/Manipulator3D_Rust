use raylib::prelude::*;

mod robot;
mod sim;
mod ui;
mod render;

use robot::{JointAngles, LinkParams, RobotArm};
use sim::LinearTrajectory;
use ui::{OverlayAction, OverlayState, OverlayStatus, UiInput};

use raylib::core::drawing::{RaylibDraw, RaylibDraw3D, RaylibMode3DExt};
use raylib::core::texture::RaylibTexture2D;
use raylib::ffi::TextureFilter;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Phase {
    MoveHomeToStart,
    PickAtStart,
    MoveStartToGoal,
    PlaceAtGoal,
    ReturnGoalToHome,
    WaitAtHomeReset,
    Error,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum BallState {
    AtStart,
    Attached,
    AtGoal,
}

#[derive(Debug)]
enum UiFont {
    Owned(Font),
    Default(WeakFont),
}

fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

fn v3_add(a: Vector3, b: Vector3) -> Vector3 {
    Vector3 {
        x: a.x + b.x,
        y: a.y + b.y,
        z: a.z + b.z,
    }
}
fn v3_sub(a: Vector3, b: Vector3) -> Vector3 {
    Vector3 {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
}
fn v3_scale(a: Vector3, s: f32) -> Vector3 {
    Vector3 {
        x: a.x * s,
        y: a.y * s,
        z: a.z * s,
    }
}
fn v3_len(a: Vector3) -> f32 {
    (a.x * a.x + a.y * a.y + a.z * a.z).sqrt()
}
fn v3_norm(a: Vector3) -> Vector3 {
    let l = v3_len(a);
    if l > 1e-6 {
        v3_scale(a, 1.0 / l)
    } else {
        Vector3 { x: 1.0, y: 0.0, z: 0.0 }
    }
}

fn segment_duration(from: Vector3, to: Vector3, speed_mps: f32) -> f32 {
    let dist = v3_len(v3_sub(to, from));
    let s = speed_mps.max(1e-3);
    // clamp so very short/very long segments still look reasonable
    clampf(dist / s, 0.35, 8.0)
}

fn load_best_ui_font(rl: &mut RaylibHandle, thread: &RaylibThread, px: i32) -> UiFont {
    let candidates = [
        "resources/fonts/Inter-Regular.ttf",
        "../resources/fonts/Inter-Regular.ttf",
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/Library/Fonts/Arial.ttf",
    ];

    for path in candidates {
        if std::path::Path::new(path).exists() {
            if let Ok(f) = rl.load_font_ex(thread, path, px, None) {
                f.texture()
                    .set_texture_filter(thread, TextureFilter::TEXTURE_FILTER_BILINEAR);
                return UiFont::Owned(f);
            }
        }
    }

    UiFont::Default(rl.get_font_default())
}

fn update_zoom(rl: &RaylibHandle, cam: &mut Camera3D) {
    let wheel = rl.get_mouse_wheel_move();
    if wheel == 0.0 {
        return;
    }

    let v = v3_sub(cam.position, cam.target);
    let mut dist = v3_len(v);
    if dist < 0.001 {
        dist = 0.001;
    }

    let mut scale = 1.0 - wheel * 0.10;
    scale = clampf(scale, 0.70, 1.30);

    dist *= scale;
    dist = clampf(dist, 1.0, 200.0);

    let dir = v3_norm(v);
    cam.position = v3_add(cam.target, v3_scale(dir, dist));
}

fn main() {
    let (mut rl, thread) = raylib::init()
        .size(1280, 720)
        .title("Manipulator3D - 3DOF IK Pick&Place (Rust)")
        .resizable()
        .msaa_4x()
        .build();

    rl.set_window_min_size(960, 540);
    rl.set_target_fps(60);

    // --- Arm parameters ---
    let mut link1 = LinkParams {
        length_m: 3.0,
        mass_kg: 2.0,
        ..Default::default()
    };
    link1.recompute_inertia();

    let mut link2 = LinkParams {
        length_m: 2.6,
        mass_kg: 1.6,
        ..Default::default()
    };
    link2.recompute_inertia();

    let arm = RobotArm::new(link1, link2);

    // Fixed EE HOME position
    let home_ee = Vector3 { x: 2.0, y: 2.0, z: 2.0 };

    // Defaults
    let mut start = Vector3 { x: 1.0, y: 2.0, z: 1.0 };
    let mut goal  = Vector3 { x: 2.0, y: 3.0, z: 2.0 };

    // UI textboxes
    let mut overlay = OverlayState::new("1 2 1", "2 3 2");

    // Camera
    let reach = arm.max_reach();
    let target = Vector3 { x: 0.0, y: 0.0, z: 0.35 * reach };
    let up     = Vector3 { x: 0.0, y: 0.0, z: 1.0 };
    let pos    = Vector3 { x: 1.10 * reach, y: -1.15 * reach, z: 0.85 * reach };
    let mut cam = Camera3D::perspective(pos, target, up, 52.0);

    // Font
    let ui_font = load_best_ui_font(&mut rl, &thread, 22);

    // Timing (fixed dwell times)
    let pick_duration = 0.45_f32;
    let place_duration = 0.35_f32;
    let reset_wait_total = 1.5_f32;

    // Motion: use constant end-effector speed (m/s) for all segments
    // Increase this number if you want the whole motion faster.
    let ee_speed_mps = 1.75_f32;

    // Runtime state
    let mut paused = true; // start paused (user enters start/goal)
    let mut phase = Phase::WaitAtHomeReset;

    let mut timer: f32 = 0.0;

    // Ball
    let ball_radius = clampf(0.03 * reach, 0.06, 0.16);
    let mut ball_state = BallState::AtStart;
    let mut ball_pos = start;

    // EE control
    let mut target_ee = home_ee;
    let mut qcmd = JointAngles::default();

    // Trajectory
    let mut traj = LinearTrajectory::default();

    let mut runtime_error: Option<String> = None;

    // Start/restart a new run using the current start/goal
    let start_simulation = |start_p: Vector3,
                            goal_p: Vector3,
                            phase_ref: &mut Phase,
                            paused_ref: &mut bool,
                            timer_ref: &mut f32,
                            target_ee_ref: &mut Vector3,
                            qcmd_ref: &mut JointAngles,
                            traj_ref: &mut LinearTrajectory,
                            ball_state_ref: &mut BallState,
                            runtime_error_ref: &mut Option<String>,
                            ee_speed_mps: f32| {
        let ik_home = arm.solve_ik(home_ee, false);
        let ik_start = arm.solve_ik(start_p, false);
        let ik_goal = arm.solve_ik(goal_p, false);

        if !ik_home.reachable || !ik_start.reachable || !ik_goal.reachable {
            *phase_ref = Phase::Error;
            *paused_ref = true;

            let mut msg = String::new();
            if !ik_home.reachable { msg.push_str("HOME not reachable. "); }
            if !ik_start.reachable { msg.push_str("START not reachable. "); }
            if !ik_goal.reachable { msg.push_str("GOAL not reachable."); }

            if msg.trim().is_empty() {
                msg = "ERROR: HOME/START/GOAL invalid or out of reach (z>=0 required).".to_string();
            }
            *runtime_error_ref = Some(msg);
            return;
        }

        *runtime_error_ref = None;

        // Initialize at HOME pose
        *qcmd_ref = ik_home.q;
        *target_ee_ref = home_ee;

        // Ball begins at START
        *ball_state_ref = BallState::AtStart;

        *timer_ref = 0.0;

        let dur = segment_duration(home_ee, start_p, ee_speed_mps);
        traj_ref.reset(home_ee, start_p, dur);

        *phase_ref = Phase::MoveHomeToStart;
        *paused_ref = false;
    };

    while !rl.window_should_close() {
        if rl.is_key_pressed(KeyboardKey::KEY_F11) {
            rl.toggle_fullscreen();
        }

        let screen_w = rl.get_screen_width();
        let screen_h = rl.get_screen_height();

        update_zoom(&rl, &mut cam);

        // UI input events
        let mouse_pos = rl.get_mouse_position();
        let mouse_click = rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT);

        let key_backspace = rl.is_key_pressed(KeyboardKey::KEY_BACKSPACE);
        let key_tab = rl.is_key_pressed(KeyboardKey::KEY_TAB);
        let key_enter = rl.is_key_pressed(KeyboardKey::KEY_ENTER);
        let key_escape = rl.is_key_pressed(KeyboardKey::KEY_ESCAPE);

        let mut chars: Vec<char> = Vec::new();
        while let Some(ch) = rl.get_char_pressed() {
            chars.push(ch);
        }

        let ui_input = UiInput {
            mouse_pos,
            mouse_click,
            chars,
            key_backspace,
            key_tab,
            key_enter,
            key_escape,
        };

        // Simulation dt
        let dt = if paused { 0.0 } else { rl.get_frame_time() };

        if phase != Phase::Error {
            match phase {
                Phase::MoveHomeToStart => {
                    ball_state = BallState::AtStart;

                    traj.update(dt);
                    target_ee = traj.position();

                    if traj.finished() {
                        phase = Phase::PickAtStart;
                        timer = 0.0;
                        target_ee = start;
                    }
                }
                Phase::PickAtStart => {
                    target_ee = start;
                    ball_state = BallState::AtStart;

                    timer += dt;
                    if timer >= pick_duration {
                        ball_state = BallState::Attached;
                        timer = 0.0;

                        let dur = segment_duration(start, goal, ee_speed_mps);
                        traj.reset(start, goal, dur);

                        phase = Phase::MoveStartToGoal;
                    }
                }
                Phase::MoveStartToGoal => {
                    traj.update(dt);
                    target_ee = traj.position();
                    ball_state = BallState::Attached;

                    if traj.finished() {
                        phase = Phase::PlaceAtGoal;
                        timer = 0.0;
                        target_ee = goal;
                    }
                }
                Phase::PlaceAtGoal => {
                    target_ee = goal;

                    timer += dt;
                    if timer >= place_duration {
                        ball_state = BallState::AtGoal;
                        timer = 0.0;

                        let dur = segment_duration(goal, home_ee, ee_speed_mps);
                        traj.reset(goal, home_ee, dur);

                        phase = Phase::ReturnGoalToHome;
                    } else {
                        ball_state = BallState::Attached;
                    }
                }
                Phase::ReturnGoalToHome => {
                    ball_state = BallState::AtGoal;

                    traj.update(dt);
                    target_ee = traj.position();

                    timer += dt; // time since place
                    if traj.finished() {
                        phase = Phase::WaitAtHomeReset;
                    }
                }
                Phase::WaitAtHomeReset => {
                    target_ee = home_ee;

                    timer += dt;
                    if timer >= reset_wait_total {
                        ball_state = BallState::AtStart;

                        timer = 0.0;

                        let dur = segment_duration(home_ee, start, ee_speed_mps);
                        traj.reset(home_ee, start, dur);

                        phase = Phase::MoveHomeToStart;
                    } else {
                        ball_state = BallState::AtGoal;
                    }
                }
                Phase::Error => {}
            }

            // IK for current target
            let ik_now = arm.solve_ik(target_ee, false);
            if !ik_now.reachable {
                phase = Phase::Error;
                paused = true;
                runtime_error = Some(ik_now.message);
            } else {
                qcmd = ik_now.q;
            }
        }

        // FK for render
        let fk = arm.forward_kinematics(qcmd);

        // Tool approach direction
        let mut approach = v3_sub(fk.ee, fk.joint2);
        let alen = v3_len(approach);
        if alen > 1e-6 {
            approach = v3_scale(approach, 1.0 / alen);
        } else {
            approach = Vector3 { x: 1.0, y: 0.0, z: 0.0 };
        }

        // Ball position (always visible)
        ball_pos = match ball_state {
            BallState::AtStart => start,
            BallState::AtGoal => goal,
            BallState::Attached => v3_add(fk.ee, v3_scale(approach, 0.22)),
        };

        // Render
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::new(10, 12, 16, 255));

        {
            let mut d3 = d.begin_mode3D(cam);

            // Thicker axes using cylinders (no grid)
            let axis_len = 3.0_f32;
            let axis_r = 0.03_f32;

            d3.draw_cylinder_ex(Vector3::zero(), Vector3 { x: axis_len, y: 0.0, z: 0.0 }, axis_r, axis_r, 12, Color::RED);
            d3.draw_cylinder_ex(Vector3::zero(), Vector3 { x: 0.0, y: axis_len, z: 0.0 }, axis_r, axis_r, 12, Color::GREEN);
            d3.draw_cylinder_ex(Vector3::zero(), Vector3 { x: 0.0, y: 0.0, z: axis_len }, axis_r, axis_r, 12, Color::BLUE);

            render::draw_robot_base_pedestal(&mut d3, Vector3::zero());
            render::draw_robot_joint_housing(&mut d3, fk.base, 0.30);
            render::draw_robot_joint_housing(&mut d3, fk.joint2, 0.24);
            render::draw_robot_joint_housing(&mut d3, fk.ee, 0.18);

            render::draw_tapered_link(&mut d3, fk.base, fk.joint2, 0.14, 0.12, Color::new(185, 185, 190, 255));
            render::draw_tapered_link(&mut d3, fk.joint2, fk.ee, 0.12, 0.10, Color::new(170, 170, 175, 255));

            render::draw_suction_tool(&mut d3, fk.ee, approach);

            d3.draw_sphere(ball_pos, ball_radius, Color::RED);
            d3.draw_sphere_wires(ball_pos, ball_radius * 1.02, 10, 10, Color::RAYWHITE);
        }

        // Overlay status
        let phase_text = match phase {
            Phase::MoveHomeToStart => "Phase: HOME -> START",
            Phase::PickAtStart => "Phase: PICK at START",
            Phase::MoveStartToGoal => "Phase: START -> GOAL (ball attached)",
            Phase::PlaceAtGoal => "Phase: PLACE at GOAL",
            Phase::ReturnGoalToHome => "Phase: GOAL -> HOME",
            Phase::WaitAtHomeReset => "Phase: WAIT then LOOP",
            Phase::Error => "Phase: ERROR",
        };

        let st = OverlayStatus {
            phase_text,
            error_text: runtime_error.as_deref(),
        };

        // Overlay + actions
        let action = match &ui_font {
            UiFont::Owned(f) => ui::draw_overlay_panel(&mut d, f, &arm, &mut overlay, &st, paused, &ui_input, screen_w, screen_h),
            UiFont::Default(f) => ui::draw_overlay_panel(&mut d, f, &arm, &mut overlay, &st, paused, &ui_input, screen_w, screen_h),
        };

        match action {
            OverlayAction::None => {}
            OverlayAction::Paused(p) => paused = p,
            OverlayAction::StartSimulation { start: ns, goal: ng } => {
                start = ns;
                goal = ng;

                start_simulation(
                    start,
                    goal,
                    &mut phase,
                    &mut paused,
                    &mut timer,
                    &mut target_ee,
                    &mut qcmd,
                    &mut traj,
                    &mut ball_state,
                    &mut runtime_error,
                    ee_speed_mps,
                );
            }
        }

        // Footer help
        match &ui_font {
            UiFont::Owned(f) => render::draw_text_small(
                &mut d,
                f,
                "F11: fullscreen   Mouse Wheel: zoom",
                12,
                screen_h - 28,
                18.0,
                Color::new(200, 200, 200, 220),
            ),
            UiFont::Default(f) => render::draw_text_small(
                &mut d,
                f,
                "F11: fullscreen   Mouse Wheel: zoom",
                12,
                screen_h - 28,
                18.0,
                Color::new(200, 200, 200, 220),
            ),
        }
    }
}
