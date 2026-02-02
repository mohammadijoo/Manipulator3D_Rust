\
<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">Manipulator3D — 3-DOF Two-Link Robot Arm (Analytic IK + Pick&amp;Place)</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Rust simulation and visualization project for a <strong>3-DOF, two-link manipulator</strong> with a <strong>fixed base</strong> in 3D space.
    The arm executes a simple <strong>pick-and-place loop</strong> using an <strong>analytic inverse kinematics</strong> solver and a <strong>linear end-effector trajectory</strong>.
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li><strong>Joint 0 (base):</strong> revolute yaw about the <strong>+Z axis</strong> at the origin <code>(0,0,0)</code>.</li>
    <li><strong>Joints 1–2 (links):</strong> revolute pitch angles defining motion in the arm’s vertical plane (relative to the <strong>X–Y plane</strong>).</li>
    <li><strong>IK:</strong> reduces the 3D target to a 2D planar problem in <code>(r,z)</code>, solves with the <strong>law of cosines</strong>, and supports elbow-up / elbow-down branches.</li>
    <li><strong>Visualization:</strong> rendered with <strong>raylib</strong> (via the <strong>raylib</strong> Rust crate), including an overlay panel and a suction-tool “ball” pick-and-place animation.</li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Build system: <strong>Cargo</strong> • Rendering: <strong>raylib</strong> • Language: <strong>Rust</strong> • IK: <strong>Closed-form</strong>
  </p>

</div>

---

<!-- ========================================================= -->
<!-- Table of Contents                                        -->
<!-- ========================================================= -->

<ul style="list-style: none; padding-left: 0; font-size: 0.97rem;">
  <li> <a href="#about-this-repository">About this repository</a></li>
  <li> <a href="#repository-structure">Repository structure</a></li>
  <li> <a href="#robotics-kinematics-dynamics-and-ik">Robotics: kinematics, dynamics, and inverse kinematics</a></li>
  <li> <a href="#building-the-project">Building the project</a></li>
  <li> <a href="#running-the-simulator">Running the simulator</a></li>
  <li> <a href="#repository-file-guide">Repository file guide (full explanation)</a></li>
  <li> <a href="#simulation-video">Simulation video</a></li>
  <li> <a href="#troubleshooting">Troubleshooting</a></li>
</ul>

---

<a id="about-this-repository"></a>

## About this repository

This project simulates a **3-DOF, two-link manipulator** mounted on a **fixed base** at the origin:

- The base joint rotates around the **Z axis** (yaw).
- The two link joints rotate as **pitch angles** (relative to the X–Y plane) in the arm’s vertical plane.
- The end-effector (EE) tracks a sequence of target points with **analytic inverse kinematics**.
- A small “ball” is used to visualize a **pick-and-place loop**:
  1) HOME → START  
  2) PICK at START (attach ball)  
  3) START → GOAL  
  4) PLACE at GOAL (detach ball)  
  5) GOAL → HOME  
  6) wait briefly and repeat  

**User inputs** (via the UI overlay panel):
- START position `(x y z)` and GOAL position `(x y z)`
- Constraint enforced by IK: **z must be ≥ 0**
- Reachability enforced by IK: `|p|` must be within the arm’s reachable shell.

**Important behavior:**
- The end-effector starts from a fixed HOME point: `(2 2 2)`.
- When paused, you can edit START/GOAL in the overlay.
- Press **PLAY** to validate inputs and start a new pick-and-place loop with the new points.

---

<a id="repository-structure"></a>

## Repository structure

```txt
Manipulator3D_Rust/
  Cargo.toml
  README.md
  .gitignore
  src/
    main.rs
    robot/
      mod.rs
      robot_arm.rs
    sim/
      mod.rs
      trajectory.rs
    ui/
      mod.rs
      overlay.rs
    render/
      mod.rs
      draw_utils.rs
  resources/
    fonts/
      (optional) Inter-Regular.ttf
```

---

<a id="robotics-kinematics-dynamics-and-ik"></a>

## Robotics: kinematics, dynamics, and inverse kinematics

### 1) Coordinate frames and joint conventions

- World frame origin is at the center of the base revolute joint: **(0,0,0)**.
- Joint angles:
  - `q0_yaw` : rotation about **+Z** (sets the arm’s radial direction in the X–Y plane)
  - `q1_pitch`: shoulder elevation angle relative to the **X–Y plane**
  - `q2_pitch`: elbow pitch angle (relative bend in the same vertical plane)

We use the radial unit direction in the X–Y plane:

```math
\mathbf{u} =
\begin{bmatrix}
\cos(q_0)\\
\sin(q_0)\\
0
\end{bmatrix},
\quad
\mathbf{k} =
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}.
```

### 2) Forward kinematics (FK)

Let link lengths be $L_1$ and $L_2$. Then:

- Elbow position:

```math
\mathbf{p}_1 = L_1\cos(q_1)\,\mathbf{u} + L_1\sin(q_1)\,\mathbf{k}
```

- End-effector position:

```math
\mathbf{p}_{ee} = \mathbf{p}_1 + L_2\cos(q_1+q_2)\,\mathbf{u} + L_2\sin(q_1+q_2)\,\mathbf{k}
```

This is implemented in:
- `robot::RobotArm::forward_kinematics()`

### 3) Workspace (reachability) check

This manipulator is effectively a 2-link arm in a vertical plane, rotated by yaw. Therefore the reachable radius must satisfy:

```math
r_{\min} = |L_1 - L_2|,\quad r_{\max} = L_1 + L_2
```

```math
\|\mathbf{p}\| \in [r_{\min}, r_{\max}]
```

The implementation enforces:
- `target.z >= 0`
- `|p|` within `[min_reach(), max_reach()]`

### 4) Analytic inverse kinematics (IK): how it works here

Given target $\mathbf{p} = (x,y,z)$:

**Step A — base yaw**

```math
q_0 = \mathrm{atan2}(y, x)
```

**Step B — reduce to planar IK in $(r,z)$**

```math
r = \sqrt{x^2 + y^2}
```

Now solve a 2-link planar problem for the triangle formed by $L_1$, $L_2$, and $\sqrt{r^2+z^2}$.

**Step C — elbow angle from law of cosines**

```math
\cos(q_2) = \frac{r^2 + z^2 - L_1^2 - L_2^2}{2L_1L_2}
```

Clamp to $[-1,1]$ for numerical safety, then:

```math
q_2 = \pm \arccos(\cos(q_2))
```

This sign is the **elbow-up / elbow-down** branch selection.

**Step D — shoulder angle**

Define:

```math
k_1 = L_1 + L_2\cos(q_2),\quad k_2 = L_2\sin(q_2)
```

Then:

```math
q_1 = \mathrm{atan2}(z, r) - \mathrm{atan2}(k_2, k_1)
```

This is implemented in:
- `robot::RobotArm::solve_ik()`

### 5) Dynamics (what is implemented vs. what is prepared)

This repository is primarily a **kinematic + trajectory** simulator:

- The EE follows a commanded Cartesian trajectory.
- IK converts EE targets into joint angles.
- FK is used for rendering joint/link positions.

However, the code also defines **mass and inertia properties** for each link (uniform rod approximations):

- About center of mass:

```math
I_{cm} = \frac{1}{12} mL^2
```

- About the joint at one end:

```math
I_{joint} = \frac{1}{3} mL^2
```

These are computed in:
- `robot::LinkParams::recompute_inertia()`

Currently, those values are used for **display/inspection** (overlay panel) and as a foundation for extending the project to:
- forward dynamics (torques → accelerations),
- gravity and Coriolis terms,
- joint-space controllers (PD, computed torque, etc.).

---

<a id="building-the-project"></a>

## Building the project

### Dependencies

- Rust toolchain (stable)
- A working C toolchain (required by raylib’s build)
- CMake (commonly required when building raylib on many platforms)

### Build commands

```bash
cargo build --release
```

---

<a id="running-the-simulator"></a>

## Running the simulator

### Run command

```bash
cargo fetch
cargo run --release
```

### Controls / interaction

- Mouse wheel: zoom camera
- F11: toggle fullscreen
- Overlay includes:
  - a **PAUSE/PLAY** button
  - START/GOAL text inputs (editable when paused)
  - reachability feedback (in/out of workspace)

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every important file in the repository and its role.

### `Cargo.toml`

Key responsibilities:

- configures the Rust crate
- depends on `raylib` for rendering and input
- uses Rust 2021 edition

---

### `src/main.rs`

This is the “application” entry point and runtime loop:

- creates the robot model (link lengths/masses)
- defines a pick-and-place finite-state machine:
  - HOME → START → PICK → GOAL → PLACE → HOME → WAIT → LOOP
- generates a **linear Cartesian trajectory** between targets
- runs IK each frame to get joint angles for the current EE target
- calls FK for rendering joint/link positions
- renders:
  - robot geometry, axes, ball, suction tool
  - overlay panel (status, parameters, pause/play, START/GOAL inputs)

---

### `src/robot/robot_arm.rs`

Declares and implements the robot model and kinematics API:

- `LinkParams`: link length, mass, inertia approximations (rod model)
- `JointAngles`: the 3 joint variables (yaw + 2 pitches)
- `FKResult`: base/joints/EE positions for rendering
- `IKResult`: reachability + solution angles + message
- `RobotArm`:
  - `solve_ik()`
  - `forward_kinematics()`
  - reach limits: `min_reach()`, `max_reach()`

---

### `src/sim/trajectory.rs`

Implements a minimal linear trajectory generator:

- `sim::LinearTrajectory`:
  - `reset(from,to,duration)`
  - `update(dt)`
  - `position()`
  - `finished()`

---

### `src/ui/overlay.rs`

Implements the overlay panel:

- draws a semi-transparent panel
- displays:
  - link lengths, masses, inertias
  - workspace bounds
  - start/goal norms and coordinates
  - phase text and reachability warnings
- implements:
  - clickable PAUSE/PLAY button
  - START/GOAL text input fields (editable while paused)

---

### `src/render/draw_utils.rs`

Rendering helpers:

- text helpers:
  - `draw_text_bold()`
  - `draw_text_small()`
- robot visuals:
  - `draw_robot_base_pedestal()`
  - `draw_robot_joint_housing()`
  - `draw_tapered_link()`
  - `draw_suction_tool()`

---

<a id="simulation-video"></a>

## Simulation video

Below is a link to the simulation video on YouTube.

<a href="https://www.youtube.com/watch?v=9-B7WbkG7cM" target="_blank">
  <img
    src="https://i.ytimg.com/vi/9-B7WbkG7cM/maxresdefault.jpg"
    alt="3D simulation of an RRR manipulator"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>

---

<a id="troubleshooting"></a>

## Troubleshooting

### Build errors mentioning C/CMake/raylib

- Ensure you have:
  - a working C toolchain (MSVC Build Tools on Windows, or gcc/clang on Linux/macOS),
  - CMake installed and on PATH.

### Window opens but UI font looks different

- If `resources/fonts/Inter-Regular.ttf` is missing, the program falls back to system fonts (or the default raylib font).
- To use Inter, place `Inter-Regular.ttf` into:
  - `resources/fonts/Inter-Regular.ttf`
