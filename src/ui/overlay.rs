use raylib::core::drawing::RaylibDraw;
use raylib::prelude::*;

use crate::render;
use crate::robot::RobotArm;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum FocusField {
    Start,
    Goal,
}

#[derive(Clone, Debug)]
pub struct UiInput {
    pub mouse_pos: Vector2,
    pub mouse_click: bool,
    pub chars: Vec<char>,
    pub key_backspace: bool,
    pub key_tab: bool,
    pub key_enter: bool,
    pub key_escape: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct OverlayStatus<'a> {
    pub phase_text: &'a str,
    pub error_text: Option<&'a str>,
}

#[derive(Debug)]
pub enum OverlayAction {
    None,
    Paused(bool),
    StartSimulation { start: Vector3, goal: Vector3 },
}

#[derive(Debug)]
pub struct OverlayState {
    start_text: String,
    goal_text: String,
    focus: FocusField,
    local_error: Option<String>,
}

impl OverlayState {
    pub fn new(start: &str, goal: &str) -> Self {
        Self {
            start_text: start.to_string(),
            goal_text: goal.to_string(),
            focus: FocusField::Start,
            local_error: None,
        }
    }
}

fn point_in_rect(p: Vector2, r: Rectangle) -> bool {
    p.x >= r.x && p.x <= r.x + r.width && p.y >= r.y && p.y <= r.y + r.height
}

fn append_chars(buf: &mut String, chars: &[char]) {
    for &ch in chars {
        if ch.is_ascii_digit()
            || ch == ' '
            || ch == '\t'
            || ch == '.'
            || ch == '-'
            || ch == '+'
            || ch == 'e'
            || ch == 'E'
        {
            buf.push(ch);
        }
    }
}

fn backspace(buf: &mut String) {
    buf.pop();
}

fn try_parse_vec3(s: &str) -> Option<Vector3> {
    let mut it = s.split_whitespace();
    let x: f32 = it.next()?.parse().ok()?;
    let y: f32 = it.next()?.parse().ok()?;
    let z: f32 = it.next()?.parse().ok()?;
    Some(Vector3 { x, y, z })
}

fn norm3(v: Vector3) -> f32 {
    (v.x * v.x + v.y * v.y + v.z * v.z).sqrt()
}

pub fn draw_overlay_panel<D, F>(
    d: &mut D,
    font: F,
    arm: &RobotArm,
    overlay: &mut OverlayState,
    status: &OverlayStatus,
    paused: bool,
    input: &UiInput,
    _screen_w: i32,
    _screen_h: i32,
) -> OverlayAction
where
    D: RaylibDraw,
    F: AsRef<raylib::ffi::Font> + Copy,
{
    // Panel geometry (taller)
    let pad = 12;
    let x0 = 14;
    let y0 = 14;
    let w = 380;
    let h = 560;

    d.draw_rectangle(x0, y0, w, h, Color::new(18, 18, 18, 230));
    d.draw_rectangle_lines(x0, y0, w, h, Color::new(200, 200, 200, 255));

    let mut y = y0 + pad;

    render::draw_text_bold(d, font, "3-DOF 2-Link Arm", x0 + pad, y, 24.0, Color::RAYWHITE);
    y += 30;

    if !status.phase_text.is_empty() {
        render::draw_text_small(d, font, status.phase_text, x0 + pad, y, 18.0, Color::SKYBLUE);
        y += 26;
    }

    // START/GOAL inputs
    let label_color = Color::new(210, 210, 210, 255);

    render::draw_text_small(d, font, "START (x y z):", x0 + pad, y, 18.0, label_color);
    y += 22;

    let start_box = Rectangle {
        x: (x0 + pad) as f32,
        y: y as f32,
        width: (w - 2 * pad) as f32,
        height: 34.0,
    };

    render::draw_text_small(d, font, "GOAL  (x y z):", x0 + pad, y + 48, 18.0, label_color);

    let goal_box = Rectangle {
        x: (x0 + pad) as f32,
        y: (y + 70) as f32,
        width: (w - 2 * pad) as f32,
        height: 34.0,
    };

    if input.mouse_click && paused {
        if point_in_rect(input.mouse_pos, start_box) {
            overlay.focus = FocusField::Start;
        } else if point_in_rect(input.mouse_pos, goal_box) {
            overlay.focus = FocusField::Goal;
        }
    }

    if input.key_tab && paused {
        overlay.focus = match overlay.focus {
            FocusField::Start => FocusField::Goal,
            FocusField::Goal => FocusField::Start,
        };
    }

    if input.key_escape && paused {
        overlay.local_error = None;
    }

    if paused {
        let buf = match overlay.focus {
            FocusField::Start => &mut overlay.start_text,
            FocusField::Goal => &mut overlay.goal_text,
        };

        if input.key_backspace {
            backspace(buf);
        }
        if !input.chars.is_empty() {
            append_chars(buf, &input.chars);
        }
    }

    let (start_border, goal_border) = match overlay.focus {
        FocusField::Start => (Color::new(120, 200, 255, 255), Color::new(180, 180, 180, 255)),
        FocusField::Goal => (Color::new(180, 180, 180, 255), Color::new(120, 200, 255, 255)),
    };

    d.draw_rectangle_rounded(start_box, 0.15, 8, Color::new(28, 28, 30, 235));
    d.draw_rectangle_rounded_lines(start_box, 0.15, 8, start_border);
    render::draw_text_small(
        d,
        font,
        overlay.start_text.as_str(),
        start_box.x as i32 + 10,
        start_box.y as i32 + 7,
        20.0,
        Color::RAYWHITE,
    );

    d.draw_rectangle_rounded(goal_box, 0.15, 8, Color::new(28, 28, 30, 235));
    d.draw_rectangle_rounded_lines(goal_box, 0.15, 8, goal_border);
    render::draw_text_small(
        d,
        font,
        overlay.goal_text.as_str(),
        goal_box.x as i32 + 10,
        goal_box.y as i32 + 7,
        20.0,
        Color::RAYWHITE,
    );

    y += 120;

    // Robot parameters block
    let l1 = arm.link1();
    let l2 = arm.link2();

    render::draw_text_small(d, font, "Link parameters:", x0 + pad, y, 18.0, Color::new(220, 220, 220, 255));
    y += 22;

    render::draw_text_small(d, font, format!("Link1 length : {:.3} m", l1.length_m).as_str(), x0 + pad, y, 18.0, Color::RAYWHITE);
    y += 20;
    render::draw_text_small(d, font, format!("Link1 mass   : {:.3} kg", l1.mass_kg).as_str(), x0 + pad, y, 18.0, Color::RAYWHITE);
    y += 20;
    render::draw_text_small(d, font, format!("Link1 inertia (joint): {:.5}", l1.inertia_joint).as_str(), x0 + pad, y, 18.0, Color::RAYWHITE);
    y += 22;

    render::draw_text_small(d, font, format!("Link2 length : {:.3} m", l2.length_m).as_str(), x0 + pad, y, 18.0, Color::RAYWHITE);
    y += 20;
    render::draw_text_small(d, font, format!("Link2 mass   : {:.3} kg", l2.mass_kg).as_str(), x0 + pad, y, 18.0, Color::RAYWHITE);
    y += 20;
    render::draw_text_small(d, font, format!("Link2 inertia (joint): {:.5}", l2.inertia_joint).as_str(), x0 + pad, y, 18.0, Color::RAYWHITE);
    y += 26;

    // Workspace + reachability
    render::draw_text_small(
        d,
        font,
        "Rule: z must be >= 0",
        x0 + pad,
        y,
        18.0,
        Color::new(200, 200, 200, 220),
    );
    y += 22;

    let rmin = arm.min_reach();
    let rmax = arm.max_reach();
    render::draw_text_small(
        d,
        font,
        format!("Workspace |p|: [{:.2}, {:.2}] m", rmin, rmax).as_str(),
        x0 + pad,
        y,
        18.0,
        Color::new(140, 200, 255, 255),
    );
    y += 24;

    let home = Vector3 { x: 2.0, y: 2.0, z: 2.0 };
    let home_ok = arm.solve_ik(home, false).reachable;

    let parsed_start = try_parse_vec3(&overlay.start_text);
    let parsed_goal = try_parse_vec3(&overlay.goal_text);

    let mut start_ok = false;
    let mut goal_ok = false;
    let mut ds = 0.0_f32;
    let mut dg = 0.0_f32;

    if let Some(s) = parsed_start {
        start_ok = arm.solve_ik(s, false).reachable;
        ds = norm3(s);
    }
    if let Some(g) = parsed_goal {
        goal_ok = arm.solve_ik(g, false).reachable;
        dg = norm3(g);
    }

    render::draw_text_small(
        d,
        font,
        format!("HOME reachable: {}", if home_ok { "YES" } else { "NO" }).as_str(),
        x0 + pad,
        y,
        18.0,
        if home_ok { Color::GREEN } else { Color::ORANGE },
    );
    y += 20;

    render::draw_text_small(
        d,
        font,
        format!("START reachable: {}   |p|={:.3}", if start_ok { "YES" } else { "NO" }, ds).as_str(),
        x0 + pad,
        y,
        18.0,
        if start_ok { Color::GREEN } else { Color::ORANGE },
    );
    y += 20;

    render::draw_text_small(
        d,
        font,
        format!("GOAL  reachable: {}   |p|={:.3}", if goal_ok { "YES" } else { "NO" }, dg).as_str(),
        x0 + pad,
        y,
        18.0,
        if goal_ok { Color::GREEN } else { Color::ORANGE },
    );
    y += 26;

    // Play/Pause button
    let btn = Rectangle {
        x: (x0 + pad) as f32,
        y: y as f32,
        width: (w - 2 * pad) as f32,
        height: 36.0,
    };

    let (btn_bg, label) = if paused {
        (Color::new(60, 120, 60, 220), "PLAY")
    } else {
        (Color::new(120, 60, 60, 220), "PAUSE")
    };

    d.draw_rectangle_rounded(btn, 0.18, 8, btn_bg);
    d.draw_rectangle_rounded_lines(btn, 0.18, 8, Color::new(230, 230, 230, 255));
    render::draw_text_bold(d, font, label, btn.x as i32 + 10, btn.y as i32 + 7, 22.0, Color::RAYWHITE);

    let pressed_btn = input.mouse_click && point_in_rect(input.mouse_pos, btn);
    let pressed_enter = input.key_enter;

    if pressed_btn || pressed_enter {
        if !paused {
            overlay.local_error = None;
            return OverlayAction::Paused(true);
        }

        let Some(s) = parsed_start else {
            overlay.local_error = Some("Invalid START format. Use: x y z".to_string());
            return OverlayAction::Paused(true);
        };
        let Some(g) = parsed_goal else {
            overlay.local_error = Some("Invalid GOAL format. Use: x y z".to_string());
            return OverlayAction::Paused(true);
        };

        let ik_home = arm.solve_ik(home, false);
        let ik_s = arm.solve_ik(s, false);
        let ik_g = arm.solve_ik(g, false);

        if !(ik_home.reachable && ik_s.reachable && ik_g.reachable) {
            overlay.local_error = Some("OUT OF REACH! Choose points inside workspace (z>=0).".to_string());
            return OverlayAction::Paused(true);
        }

        overlay.local_error = None;
        return OverlayAction::StartSimulation { start: s, goal: g };
    }

    // Error lines
    y += 48;
    if let Some(e) = status.error_text {
        render::draw_text_bold(d, font, e, x0 + pad, y, 18.0, Color::RED);
        y += 22;
    }
    if let Some(e) = overlay.local_error.as_deref() {
        render::draw_text_bold(d, font, e, x0 + pad, y, 18.0, Color::RED);
    }

    OverlayAction::None
}
