<div style="font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; line-height: 1.6;">

  <h1 align="center" style="margin-bottom: 0.2em;">Manipulator3D — 3-DOF Two-Link Robot Arm (Analytic IK + Pick&amp;Place) — Rust</h1>

  <p style="font-size: 0.98rem; max-width: 920px; margin: 0.2rem auto 0;">
    A Rust simulation and visualization project for a <strong>3-DOF, two-link manipulator</strong> with a <strong>fixed base</strong> in 3D space.
    The arm executes a simple <strong>pick-and-place loop</strong> using an <strong>analytic inverse kinematics</strong> solver and a <strong>linear end-effector trajectory</strong>.
  </p>

  <ul style="max-width: 920px; margin: 0.5rem auto 0.2rem; padding-left: 1.2rem;">
    <li><strong>Joint 0 (base):</strong> revolute yaw about the <strong>+Z axis</strong> at the origin <code>(0,0,0)</code>.</li>
    <li><strong>Joints 1–2 (links):</strong> revolute pitch angles defining motion in the arm’s vertical plane (relative to the <strong>X–Y plane</strong>).</li>
    <li><strong>IK:</strong> reduces the 3D target to a 2D planar problem in <code>(r,z)</code>, solves with the <strong>law of cosines</strong>, and supports elbow-up / elbow-down branches.</li>
    <li><strong>Visualization:</strong> rendered with <strong>raylib</strong>, including an overlay panel and a simple suction-tool “ball” pick-and-place animation.</li>
    <li><strong>User input:</strong> START/GOAL are entered in the <strong>UI overlay</strong> (no console prompts). Press <strong>PLAY</strong> to start; press <strong>PAUSE</strong> to edit inputs.</li>
  </ul>

  <p align="center" style="font-size: 1rem; color: #666; margin: 0.35rem auto 0;">
    Build system: <strong>Cargo</strong> • Rendering: <strong>raylib</strong> • Language: <strong>Rust (Edition 2021)</strong> • IK: <strong>Closed-form</strong>
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
  <li> <a href="#operating-system-guides-windows--macos--linux">Operating system guides (Windows / macOS / Linux)</a></li>
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

**User inputs** (via overlay panel):
- START position `(x y z)` and GOAL position `(x y z)`
- Constraint enforced by IK: **z must be ≥ 0**
- Reachability enforced by IK: `|p|` must be within the arm’s reachable shell.

**Controls / interaction**
- Mouse wheel: zoom camera
- F11: toggle fullscreen
- Overlay includes a **PAUSE/PLAY** button, editable START/GOAL fields, and reachability feedback

---

<a id="repository-structure"></a>

## Repository structure

```txt
Manipulator3D_Rust/
  Cargo.toml
  Cargo.lock
  README.md
  .cargo/
    config.toml
  resources/
    fonts/
      Inter-Regular.ttf
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
```

### About `.cargo/config.toml` (important)

This repository includes a `.cargo/config.toml` tuned for **slow/unstable networks** and environments where Cargo downloads time out.
If your network is stable, you can keep it as-is (it does not change runtime behavior), or adjust the values.

Typical improvements it provides:
- avoids flaky HTTP/2 multiplexing
- increases timeouts/retries
- relaxes low-speed limits for very slow links
- uses the `git` crates.io index protocol to avoid sparse-index issues

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
- `|p|` within `[MinReach(), MaxReach()]`

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

---

<a id="building-the-project"></a>

## Building the project

### Dependencies

- Rust toolchain (Rustup recommended)
- A native C/C++ toolchain (raylib is built natively via `raylib-sys`)
  - Windows: Visual Studio Build Tools / MSVC
  - macOS: Xcode Command Line Tools
  - Linux: build-essential (GCC/Clang + make)

### Build commands (recommended)

From the repository root (where `Cargo.toml` is):

```bash
cargo clean
cargo build --release
```

---

<a id="operating-system-guides-windows--macos--linux"></a>

## Operating system guides (Windows / macOS / Linux)

### Windows 10/11 (MSVC toolchain)

This project targets the <strong>MSVC</strong> Rust toolchain on Windows. The most reliable approach is to build from the
<strong>Developer PowerShell for VS 2022</strong> (or the <strong>x64 Native Tools Command Prompt</strong>), because it pre-configures the
MSVC compiler + linker environment.

#### Recommended workflow (Developer PowerShell)

1) Install prerequisites:
<ul>
  <li><strong>Rust</strong> (Rustup) using the <code>x86_64-pc-windows-msvc</code> toolchain</li>
  <li><strong>Visual Studio 2022 Build Tools</strong> with the workload: <em>Desktop development with C++</em></li>
  <li><strong>Windows 10/11 SDK</strong> (installed with the Build Tools when selected)</li>
</ul>

2) Open: <strong>Developer PowerShell for VS 2022</strong>.

3) Build and run:
<pre><code>cd path\to\Manipulator3D_Rust
cargo fetch
cargo run --release</code></pre>

#### If you see linker errors (kernel32.lib not found / wrong link.exe)

Some Windows environments (especially when MSYS2 / Git Bash is installed) can accidentally pick up a non-MSVC <code>link.exe</code>.
If you see errors referencing paths like <code>C:\msys64\usr\bin\link.exe</code>, you can explicitly configure the MSVC libraries and linker
inside PowerShell before building.

Run the following PowerShell commands (edit the version/path to match your machine):

<pre><code># 1) MSVC version you have (taken from your MSVC folder name)
#    Change this to match your system.
#    How to find it:
#      Open:   C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\
#      Pick the newest folder name (example: 14.44.35207)
$msvcVer = "14.44.35207"

# 2) MSVC lib directory (x64)
$msvcLib = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\$msvcVer\lib\x64"

# 3) Windows SDK (Windows Kits) library directories
$sdkRoot = "C:\Program Files (x86)\Windows Kits\10\Lib"
$sdkVer  = (Get-ChildItem $sdkRoot -Directory | Sort-Object Name -Descending | Select-Object -First 1).Name

$umLib   = Join-Path $sdkRoot "$sdkVer\um\x64"
$ucrtLib = Join-Path $sdkRoot "$sdkVer\ucrt\x64"

# 4) Sanity checks (should print True / True)
Test-Path (Join-Path $umLib "kernel32.lib")
Test-Path (Join-Path $msvcLib "msvcrt.lib")

# 5) Tell the linker where to find .lib files
$env:LIB = "$msvcLib;$ucrtLib;$umLib;$env:LIB"

# 6) Force Cargo to use MSVC's linker (NOT MSYS2/Git's link.exe)
$env:CARGO_TARGET_X86_64_PC_WINDOWS_MSVC_LINKER = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64\link.exe"

# 7) Build and run from the repository root (where Cargo.toml is)
cd "$env:USERPROFILE\Desktop\Manipulator3D_Rust"

cargo clean
cargo fetch
cargo run --release
</code></pre>

### macOS

1) Install Rust (or verify it is installed):
<pre><code>rustc --version
cargo --version</code></pre>

2) Ensure build tools are available (Xcode Command Line Tools):
<pre><code>xcode-select --install</code></pre>

3) Run:
<pre><code>cd path/to/Manipulator3D_Rust
cargo fetch
cargo run --release</code></pre>

### Linux (Ubuntu/Debian)

1) Install Rust (or verify it is installed):
<pre><code>rustc --version
cargo --version</code></pre>

2) Install native build dependencies:
<pre><code>sudo apt-get update
sudo apt-get install -y build-essential pkg-config</code></pre>

3) Run:
<pre><code>cd path/to/Manipulator3D_Rust
cargo fetch
cargo run --release</code></pre>

---

<a id="running-the-simulator"></a>

## Running the simulator

From the repository root:

```bash
cargo fetch
cargo run --release
```

### Controls / interaction

- Mouse wheel: zoom camera
- F11: toggle fullscreen
- Overlay:
  - edit START and GOAL when paused
  - press PLAY to start the simulation
  - press PAUSE to stop and edit inputs again

---

<a id="repository-file-guide"></a>

## Repository file guide (full explanation)

This section explains every important file in the repository and its role.

### `Cargo.toml`

- Declares the package and dependencies.
- Uses `raylib` crate (which pulls `raylib-sys` for native compilation).

---

### `.cargo/config.toml`

- Cargo networking configuration tuned for slow/unstable connections.
- Useful when downloads repeatedly timeout or reset.

---

### `src/main.rs`

Runtime loop + simulation state machine:

- reads START/GOAL from overlay input fields
- validates reachability using IK for:
  - fixed HOME EE point
  - START
  - GOAL
- runs a pick-and-place finite-state machine:
  - HOME → START → PICK → GOAL → PLACE → HOME → WAIT → LOOP
- generates a linear Cartesian trajectory between targets
- runs IK each frame to get joint angles for the current EE target
- calls FK for rendering joint/link positions
- renders:
  - robot geometry, axes, ball, suction tool
  - overlay panel (inputs, parameters, status, play/pause)

---

### `src/robot/robot_arm.rs`

Implements analytic FK and IK:

- IK:
  - rejects invalid targets (z < 0)
  - checks radius bounds (min/max reach)
  - computes yaw from `atan2(y,x)`
  - reduces to planar IK in `(r,z)`
  - solves elbow angle via law of cosines
  - solves shoulder angle via triangle decomposition
- FK:
  - constructs radial axis from yaw
  - builds elbow and EE positions from link lengths and pitch angles

---

### `src/sim/trajectory.rs`

Minimal trajectory generator:

- linear interpolation with normalized time:
  - $\alpha = \mathrm{clamp}(t/T, 0, 1)$
- position:
  - $\mathbf{p}(\alpha) = \mathbf{a} + (\mathbf{b}-\mathbf{a})\alpha$

---

### `src/ui/overlay.rs`

Overlay rendering and interaction:

- editable START and GOAL input boxes
- reachability checks and status display
- PLAY/PAUSE button:
  - PLAY validates inputs and starts a new simulation
  - PAUSE stops simulation so inputs can be edited

---

### `src/render/draw_utils.rs`

Rendering helpers:

- text helpers (small and bold)
- robot visuals:
  - base pedestal + flange
  - joint housings
  - tapered links
  - suction tool

---

<a id="simulation-video"></a>

## Simulation video

Below is a link to the simulation video on YouTube.

<a href="https://www.youtube.com/watch?v=9-B7WbkG7cM" target="_blank">
  <img
    src="https://i.ytimg.com/vi/9-B7WbkG7cM/maxresdefault.jpg"
    alt="3D simulation of an RRR manipulator written in Rust"
    style="max-width: 100%; border-radius: 10px; box-shadow: 0 6px 18px rgba(0,0,0,0.18); margin-top: 0.5rem;"
  />
</a>

---

<a id="troubleshooting"></a>

## Troubleshooting

### Cargo downloads are extremely slow or fail with timeouts/resets

- This repo includes `.cargo/config.toml` to improve reliability on slow/unstable networks.
- If you still see timeouts:
  - try again later or from a different network
  - ensure proxies/VPNs are not interfering with `static.crates.io`

### Build fails on Windows with linker errors

- Use **Developer PowerShell for VS 2022**.
- If `kernel32.lib` is missing or a wrong `link.exe` is used, follow the PowerShell configuration in:
  - <a href="#windows-1011-msvc-toolchain">Windows 10/11 (MSVC toolchain)</a>

### Build fails with `libclang` / bindgen related errors

Some systems require LLVM/Clang to be installed for `bindgen`/`clang-sys` when building native dependencies.

- Windows: install LLVM and ensure `libclang` is discoverable
- macOS: Xcode Command Line Tools usually provide clang
- Linux: install clang/llvm packages if needed

### Start/Goal is “out of reach”

- Ensure:
  - `z >= 0`
  - `|p|` is within the workspace shell:
    - `[|L1 - L2|, L1 + L2]`
