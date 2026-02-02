use raylib::core::drawing::{RaylibDraw, RaylibDraw3D};
use raylib::prelude::*;

fn draw_text_ex_at<D, F>(d: &mut D, font: F, text: &str, x: i32, y: i32, font_size: f32, color: Color)
where
    D: RaylibDraw,
    F: AsRef<raylib::ffi::Font> + Copy,
{
    d.draw_text_ex(
        font,
        text,
        Vector2 { x: x as f32, y: y as f32 },
        font_size,
        1.0,
        color,
    );
}

pub fn draw_text_bold<D, F>(d: &mut D, font: F, text: &str, x: i32, y: i32, font_size: f32, color: Color)
where
    D: RaylibDraw,
    F: AsRef<raylib::ffi::Font> + Copy,
{
    draw_text_ex_at(d, font, text, x, y, font_size, color);
    draw_text_ex_at(d, font, text, x + 1, y, font_size, color);
    draw_text_ex_at(d, font, text, x, y + 1, font_size, color);
    draw_text_ex_at(d, font, text, x + 1, y + 1, font_size, color);
}

pub fn draw_text_small<D, F>(d: &mut D, font: F, text: &str, x: i32, y: i32, font_size: f32, color: Color)
where
    D: RaylibDraw,
    F: AsRef<raylib::ffi::Font> + Copy,
{
    draw_text_ex_at(d, font, text, x, y, font_size, color);
}

pub fn draw_robot_base_pedestal<D: RaylibDraw3D>(d: &mut D, origin: Vector3) {
    let a = Vector3 { x: origin.x, y: origin.y, z: -0.25 };
    let b = Vector3 { x: origin.x, y: origin.y, z:  0.00 };
    d.draw_cylinder_ex(a, b, 0.55, 0.55, 24, Color::new(70, 70, 75, 255));

    let c = Vector3 { x: origin.x, y: origin.y, z: 0.00 };
    let d2 = Vector3 { x: origin.x, y: origin.y, z: 0.35 };
    d.draw_cylinder_ex(c, d2, 0.38, 0.34, 24, Color::new(95, 95, 100, 255));

    d.draw_cylinder_ex(
        Vector3 { x: origin.x, y: origin.y, z: 0.00 },
        Vector3 { x: origin.x, y: origin.y, z: 0.06 },
        0.48,
        0.48,
        24,
        Color::new(110, 110, 115, 255),
    );
}

pub fn draw_robot_joint_housing<D: RaylibDraw3D>(d: &mut D, center: Vector3, radius: f32) {
    d.draw_sphere(center, radius, Color::new(120, 120, 125, 255));
    d.draw_sphere_wires(center, radius, 12, 12, Color::new(200, 200, 200, 60));
    d.draw_cylinder_ex(
        Vector3 { x: center.x, y: center.y, z: center.z - 0.10 },
        Vector3 { x: center.x, y: center.y, z: center.z + 0.10 },
        radius * 0.55,
        radius * 0.55,
        18,
        Color::new(85, 85, 90, 255),
    );
}

pub fn draw_tapered_link<D: RaylibDraw3D>(d: &mut D, a: Vector3, b: Vector3, r_a: f32, r_b: f32, color: Color) {
    d.draw_cylinder_ex(a, b, r_a, r_b, 20, color);
    d.draw_sphere(a, r_a * 0.95, Color::new(140, 140, 145, 255));
    d.draw_sphere(b, r_b * 0.95, Color::new(140, 140, 145, 255));
}

pub fn draw_suction_tool<D: RaylibDraw3D>(d: &mut D, ee: Vector3, approach_dir: Vector3) {
    let tip = Vector3 {
        x: ee.x + approach_dir.x * 0.28,
        y: ee.y + approach_dir.y * 0.28,
        z: ee.z + approach_dir.z * 0.28,
    };
    d.draw_cylinder_ex(ee, tip, 0.06, 0.05, 18, Color::new(40, 40, 45, 255));

    let cup_a = tip;
    let cup_b = Vector3 {
        x: tip.x + approach_dir.x * 0.06,
        y: tip.y + approach_dir.y * 0.06,
        z: tip.z + approach_dir.z * 0.06,
    };
    d.draw_cylinder_ex(cup_a, cup_b, 0.11, 0.11, 24, Color::new(25, 25, 28, 255));
    d.draw_sphere(tip, 0.035, Color::new(80, 80, 85, 255));
}
