# Architecture Deep-Dive

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        index.html                                    │
│  ┌──────────────────────────────┬────────────────────────────────┐   │
│  │   p5.js Canvas               │   Dashboard Panel              │   │
│  │   (warehouse + AGV sim)      │   - Q/R sliders               │   │
│  │                              │   - Slip mode selector        │   │
│  │                              │   - Live metrics              │   │
│  │                              │   - Savings calculator        │   │
│  │                              │   - Sales narrative            │   │
│  ├──────────────────────────────┴────────────────────────────────┤   │
│  │   Explanatory Accordion (Executive / Engineer / Specialist)    │   │
│  └─────────────────────────────────────────────────────────────────┘  │
│                                                                       │
│  CDN: p5.js | math.js | KaTeX        Test: Mocha + Chai + math.js    │
└─────────────────────────────────────────────────────────────────────┘
```

The application is a single-page, client-only web app with no backend. All logic runs in the browser.

---

## 2. Data Flow

### 2.1 Simulation Loop (60 fps via p5.js `draw()`)

```
p5.js draw()
    │
    ├─ Check state: completed/crashed? → render final scene
    ├─ Check: running? → render idle prompt
    ├─ Check: simTime ≥ TOTAL_TIME? → routeComplete()
    │
    ├─ 1. Get true state from trajectory
    ├─ 2. Get corrupted control (slip applied in corridor)
    ├─ 3. EKF.predict(control, dt)
    ├─ 4. Get visible landmarks (empty in corridor)
    ├─ 5. If landmark: generate noisy measurement → EKF.update(meas, lm)
    ├─ 6. Compute divergence & uncertainty
    ├─ 7. Check collision (EKF estimate vs. rack bounding boxes)
    ├─ 8. Check off-canvas (EKF estimate out of bounds)
    ├─ 9. Push to history arrays
    ├─ 10. Render: racks, landmarks, path, true/ekf trails, measurements
    ├─ 11. Update dashboard metrics
    └─ 12. simTime += DT
```

### 2.2 State Machine

```
   IDLE ──(tune sliders + select mode)──→ READY
                                           │
                                     (press DEPLOY)
                                           │
                                           ▼
                                       RUNNING
                                           │
                             ┌─────────────┼─────────────┐
                             │             │             │
                             ▼             ▼             ▼
                      CORRIDOR_ENTRY  CORRIDOR_EXIT  (divergence)
                        (no landmarks)  (landmarks     │
                         + slip)         return)       │
                             │             │           │
                             └──────┬──────┘           │
                                    │                  │
                                    ▼                  ▼
                             (recovery)           DIVERGED
                                    │                  │
                                    ▼                  ▼
                             SUCCESS            GAME_OVER
                                    │                  │
                                    └──────┬───────────┘
                                           │
                                     (press TUNE AGAIN)
                                           │
                                           ▼
                                         IDLE
```

---

## 3. EKF Implementation (`script.js:4–134`)

### 3.1 State Vector

The filter uses a 4-dimensional state for planar pose and velocity:

$$\mathbf{x}_k = \begin{bmatrix} x \\ y \\ \theta \\ v \end{bmatrix}$$

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | `x` | Global X position | pixels |
| 1 | `y` | Global Y position | pixels |
| 2 | `θ` | Heading (yaw) | radians |
| 3 | `v` | Forward velocity | pixels/timestep |

### 3.2 Control Input

$$\mathbf{u}_k = \begin{bmatrix} \dot{\theta} \\ a \end{bmatrix}$$

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | `θ̇` | Steering rate | rad/timestep |
| 1 | `a` | Acceleration | px/timestep² |

### 3.3 Motion Model (Non-Linear)

$$\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k) = \begin{bmatrix} x_k + v_k \cos(\theta_k) \Delta t \\ y_k + v_k \sin(\theta_k) \Delta t \\ \theta_k + \dot{\theta} \Delta t \\ v_k + \dot{v} \Delta t \end{bmatrix}$$

This is a constant-velocity bicycle model with steering input.

### 3.4 Jacobian of Motion Model (4×4)

$$\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}} \bigg|_{\mathbf{x}_{k-1}, \mathbf{u}_k}$$

$$\mathbf{F}_k = \begin{bmatrix} 1 & 0 & -v_k \sin(\theta_k) \Delta t & \cos(\theta_k) \Delta t \\ 0 & 1 & v_k \cos(\theta_k) \Delta t & \sin(\theta_k) \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### 3.5 Predict Step

$$\mathbf{x}_k^- = f(\mathbf{x}_{k-1}, \mathbf{u}_k)$$

$$\mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^\mathsf{T} + \mathbf{Q}$$

### 3.6 Measurement Model (Range-Bearing)

Measurements are taken to the nearest visual landmark:

$$\mathbf{z}_k = h(\mathbf{x}_k, l) = \begin{bmatrix} \sqrt{(l_x - x_k)^2 + (l_y - y_k)^2} \\ \text{atan2}(l_y - y_k, l_x - x_k) - \theta_k \end{bmatrix}$$

### 3.7 Jacobian of Measurement Model (2×4)

$$\mathbf{H}_k = \frac{\partial h}{\partial \mathbf{x}} \bigg|_{\mathbf{x}_k^-}$$

$$\mathbf{H}_k = \begin{bmatrix} -\frac{\Delta x}{d} & -\frac{\Delta y}{d} & 0 & 0 \\ \frac{\Delta y}{d^2} & -\frac{\Delta x}{d^2} & -1 & 0 \end{bmatrix}$$

where Δx = l_x − x, Δy = l_y − y, d = √(Δx² + Δy²)

### 3.8 Update Step

$$\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^\mathsf{T} \left( \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^\mathsf{T} + \mathbf{R} \right)^{-1}$$

$$\mathbf{x}_k^+ = \mathbf{x}_k^- + \mathbf{K}_k (\mathbf{z}_k - h(\mathbf{x}_k^-))$$

$$\mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^-$$

### 3.9 Covariance Matrices

| Matrix | Dimension | Description | Default | Slider Range |
|--------|-----------|-------------|---------|--------------|
| `Q` | 4×4 diagonal | Process noise (trust model) | diag(0.1, 0.1, 0.05, 0.02) | 0.01 – 5.0× default |
| `R` | 2×2 diagonal | Measurement noise (trust sensors) | diag(0.5, 0.5) | 0.01 – 5.0× default |
| `P` | 4×4 symmetric | State covariance (evolves) | 0.1 × I₄ | — |

---

## 4. Simulation Mechanics

### 4.1 Trajectory

The reference path is a cubic Bézier curve from Point A (150, 440) to Point B (700, 160) with control points at (280, 440) and (500, 300). The trajectory is pre-computed at integer timesteps and interpolated with linear interpolation between steps.

400 timesteps at ~60 fps ≈ 6.7 seconds per run.

### 4.2 The Corridor Trap

| Aspect | Implementation |
|--------|---------------|
| **Timing** | Default `slipStartTime` = 120, `slipEndTime` = 280 (timesteps 120–280 of 400) |
| **Landmark dropout** | `getVisibleLandmarks()` returns empty array when `isInCorridor(simTime)` is true |
| **Control corruption** | `getCorruptedControl()` subtracts a sinusoidal steering bias (max 0.015 rad/timestep) from the nominal `thetaDot` during the corridor window |
| **Noise** | Small random noise (±0.002 rad/timestep) added to prevent exact reproducibility |

### 4.3 Slip Mode

- **Deterministic**: Slip always occurs at timesteps 120–280 (fixed).
- **Semi-Random**: `computeSlipParams()` randomizes the onset (±60 steps) and offset (±40 steps) of the corridor window. Called at deploy time.

### 4.4 Collision Detection

```javascript
function checkCollision() {
  // Simple AABB overlap test with ROBOT_RADIUS margin
  // EKF estimate (x, y) vs. each rack bounding box
}
```

Racks are placed along the corridor exit (timesteps ~250–350) where EKF divergence is maximal after the landmark-free zone.

---

## 5. Hardware Savings Calculator

```
savings = 0.40 * max(0, 1 - σ / σ_max)

where:
  σ     = sqrt(tr(P[0:2, 0:2]))  — position standard deviation from EKF
  σ_max = 3.0                     — threshold where savings = 0
  0.40  = 40% maximum savings
```

The dollar savings displayed in the dashboard use a hypothetical $5,000 lidar cost:

```
dollar_savings = round(savings * 5000)
```

---

## 6. Rendering Pipeline

### 6.1 Layer Order (back to front)

```
1. Floor + grid
2. Trajectory path (faded)
3. Racks (with hazard stripes)
4. Landmarks (with sensor range circles)
5. Start/End zones (A/B markers)
6. True position trail (ghost grey polyline)
7. EKF estimate trail (blue polyline)
8. Corridor zone warning overlay
9. Raw sensor measurement (red dot + landmark line)
10. True AGV (ghost, 25% opacity)
11. EKF AGV (solid blue, direction indicator)
12. Overlays (Game Over / Success) — positioned via CSS
```

---

## 7. File Architecture

| File | Responsibility |
|------|---------------|
| `index.html` | DOM structure, CSS (~480 lines), CDN script tags, 3-tier accordion with KaTeX |
| `script.js` | `EKFSlam` class (131 lines), simulation world (112 lines), p5.js setup/draw (92 lines), rendering (280 lines), UI/dashboard (170 lines), Node.js exports (20 lines) |
| `tests/setup.js` | Provides `global.math` from npm `mathjs` package |
| `tests/test_ekf.js` | 28 unit tests for the EKF class |
| `tests/test_functions.js` | 17 unit tests for pure simulation functions |
| `Dockerfile` | Multi-stage nginx:alpine container |
| `.github/workflows/deploy.yml` | GitHub Actions → Pages with CI gate |

---

## 8. Test Coverage

### EKF Unit Tests (28 tests)

| Group | Tests | Coverage |
|-------|-------|----------|
| Constructor | 4 | State, P, Q, R initialization |
| State accessors | 1 | setState + getters |
| normalizeAngle | 3 | Range, preservation, wrapping |
| Motion model `f()` | 4 | Straight, steering, acceleration, diagonal |
| Jacobian F | 2 | Dimensions, structure |
| Predict | 3 | Uncertainty growth, state change, angle normalization |
| Measurement model `h()` | 2 | Range/bearing, relative bearing |
| Jacobian H | 2 | Dimensions, degenerate case |
| Update | 2 | Uncertainty reduction, convergence |
| setTuning | 3 | Q scaling, R scaling, default immutability |
| Uncertainty | 2 | Trace computation, non-negativity |
| Integration | 1 | 50-step stability |

### Pure Function Tests (17 tests)

| Group | Tests | Coverage |
|-------|-------|----------|
| cubicBezier | 3 | t=0, t=1, interpolation |
| measurementToPosition | 2 | Null handling, Cartesian conversion |
| isInCorridor | 2 | Inside/outside corridor |
| calculateSavings | 4 | Perfect estimate, max uncertainty, monotonicity, bounds |
| getCorruptedControl | 2 | Return structure, acceleration = 0 |
| noisyMeasurement | 2 | Return structure, unbiased noise |

### Headless Browser Integration Test (1 test, 15 assertions)

Run via `npm run test:headless` using Playwright + Firefox. Validates the full application lifecycle in a real browser environment:

| Phase | Assertions |
|-------|------------|
| Page load | Canvas exists, visible, and has rendered content |
| p5.js init | `setup()` and `draw()` are defined globally |
| UI state | Slider defaults correct, mode selector active, start button enabled |
| Click "DEPLOY AGV" | `running` flips to `true`, button text updates, simTime advances |
| Mid-simulation | EKF state coordinates are non-zero and plausible |
| Route completion | `completed = true`, `crashed = false`, no JS errors throughout |

---

## 9. CI/CD Pipeline

Two GitHub Actions workflows enforce a strict quality gate before any deployment reaches GitHub Pages.

### Workflow: `test.yml` (PR & push validation)

Triggers: every pull request and push to `main`/`master`.

```yaml
jobs:
  unit:        # 45 Mocha tests — EKF math + pure functions
  e2e:         # Playwright + Firefox headless integration
    needs: unit
```

### Workflow: `deploy.yml` (production deployment)

Triggers: push to `main`/`master` or manual `workflow_dispatch`.

```
       ┌──────────────┐
       │  Push to main │
       └──────┬───────┘
              ▼
    ┌─────────────────┐
    │  Unit Tests (45) │─── fail ──→ ❌ Blocked
    └────────┬────────┘
             ▼ pass
    ┌─────────────────────────┐
    │  Headless Browser Test  │─── fail ──→ ❌ Blocked
    └────────┬────────────────┘
             ▼ pass
    ┌──────────────────────┐
    │  Deploy to Pages     │───→ 🌐 Live
    └──────────────────────┘
```

### Headless Browser Integration Test

The e2e test (`tests/headless_test.js`) uses **Playwright** with **Firefox** in headless mode to verify:

| Check | What it validates |
|-------|-------------------|
| Canvas renders | `document.querySelectorAll('canvas').length > 0` and the canvas has non-zero pixel content |
| p5.js initialised | `typeof setup === 'function' && typeof draw === 'function'` |
| Slider defaults | Q=1.0 and R=1.0 at initial state |
| Mode selector | Deterministic mode is active by default |
| Start button | Enabled, labeled "▶ DEPLOY AGV" |
| Click triggers run | After clicking, `running = true`, `simTime` advances |
| Simulation progresses | EKF state (`ekf.getX()`, `ekf.getY()`) updates with non-zero coordinates |
| Route completion | Simulation reaches `t = TOTAL_TIME` with `completed = true` and no crashes |
| Zero JS errors | No `console.error`, `pageerror`, or `requestfailed` events during the entire lifecycle |

Screenshots at four key moments (initial, after click, mid-simulation, final) are captured and uploaded as artifacts on failure for debugging.

### Required status checks

For branch protection, configure:

- **"Unit Tests"** — required
- **"Headless Browser Tests"** — required
- **"Deploy to GitHub Pages"** — (optional, only runs on main)

This ensures every PR passes both static analysis (Mocha) and a live browser rendering test before merge.
