# Architecture Deep-Dive

## 1. System Overview

```
┌─────────────────────────────────┬──────────────────────────────────┐
│   Left Column (3fr)            │ Right Column: Dashboard (2fr)     │
│  ┌───────────────────────────┐ │ ┌──────────────────────────────┐  │
│  │  p5.js Canvas             │ │ │  RUN SIMULATION Button       │  │
│  │  (warehouse + AGV sim)    │ │ │  AGV Speed Slider           │  │
│  │  GPS/IMU indicators       │ │ │  + A/B coordinate display   │  │
│  ├───────────────────────────┤ │ │  Stop on Divergence Toggle  │  │
│  │  Result Panel             │ │ │  ▼ Sensors Config (coll)    │  │
│  │  (game-over / success)    │ │ │  ├ Interior (odometry, IMU) │  │
│  ├───────────────────────────┤ │ │  └ Exterior (LIDAR, GPS,   │  │
│  │  ▼ EKF Accordion (coll)  │ │ │       beacons + range)      │  │
│  │  ├ Executive              │ │ │  ▼ Route Config (coll)      │  │
│  │  ├ Engineer               │ │ │  ├ A/B drag markers        │  │
│  │  └ Specialist (KaTeX)     │ │ │  └ Slip mode selector      │  │
│  │    + symbol legend spoiler│ │ │  ▼ Live Metrics (coll)      │  │
│  └───────────────────────────┘ │ │  ▼ Hardware Savings (coll) │  │
│                                │ │  ▼ Business Impact (coll)  │  │
│                                │ └──────────────────────────────┘  │
└─────────────────────────────────┴──────────────────────────────────┘

CDN: p5.js | math.js | KaTeX        Tests: Mocha + Chai + Playwright
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
    ├─ 3. EKF.predict(control, dt) (odometry-based, always runs)
    ├─ 3b. If IMU enabled: noisy θ and v → EKF.imuUpdate() (superimposed on odometry, always available interior)
    ├─ 4. Get visible landmarks (empty in corridor)
    ├─ 5. If landmark + LIDAR enabled: noisy measurement → EKF.update(meas, lm)
    ├─ 6. If GPS enabled + outside corridor: noisy GPS → EKF.gpsUpdate()
    ├─ 7. For each external sensor (beacon): if within range + LIDAR enabled + outside corridor: direct position update → EKF.gpsUpdate()
    ├─ 8. Compute divergence & uncertainty
    ├─ 9. Check divergence (>60px from true) → gameOver('divergence')
    ├─ 10. Check collision (EKF estimate vs. rack bounding boxes) → gameOver('collision')
    ├─ 11. Check off-canvas (EKF estimate out of bounds) → gameOver('lost')
    ├─ 12. Push to history arrays (up to 2000 points)
    ├─ 13. Render: racks, landmarks (highlighted active beacons), path, true/ekf trails, measurements
    ├─ 14. Update dashboard metrics
    └─ 15. simTime += DT × (agvSpeed / 2.0)
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

## 3. EKF Implementation

### 3.1 State Vector

The filter uses a 4-dimensional state for planar pose and velocity:

$$\mathbf{x}_k = \begin{bmatrix} x_k\\\\ y_k\\\\ \theta_k\\\\ v_k \end{bmatrix}$$

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | `x` | Global X position | pixels |
| 1 | `y` | Global Y position | pixels |
| 2 | `θ` | Heading (yaw) | radians |
| 3 | `v` | Forward velocity | pixels/timestep |

### 3.2 Control Input

$$\mathbf{u}_k = \begin{bmatrix} \dot{\theta}_k\\\\ a_k \end{bmatrix}$$

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | `θ̇` | Steering rate | rad/timestep |
| 1 | `a` | Acceleration | px/timestep² |

### 3.3 Motion Model (Non-Linear)

$$\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k) = \begin{bmatrix} x_k + v_k \cos(\theta_k) \Delta t\\\\ y_k + v_k \sin(\theta_k) \Delta t\\\\ \theta_k + \dot{\theta} \Delta t\\\\ v_k + \dot{v} \Delta t \end{bmatrix}$$

This is a bicycle model with steering input.

### 3.4 Jacobian of Motion Model (4×4)

$$\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}} \bigg|_{\mathbf{x}_{k-1}, \mathbf{u}_k}$$

$$\mathbf{F}_k = \begin{bmatrix} 1 & 0 & -v_{k-1} \sin(\theta_{k-1}) \Delta t & \cos(\theta_{k-1}) \Delta t\\\\ 0 & 1 & v_{k-1} \cos(\theta_{k-1}) \Delta t & \sin(\theta_{k-1}) \Delta t\\\\ 0 & 0 & 1 & 0\\\\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### 3.5 Predict Step

$$\mathbf{x}_k^- = f(\mathbf{x}_{k-1}, \mathbf{u}_k)$$

$$\mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^\mathsf{T} + \mathbf{Q}$$

### 3.6 Measurement Model (Range-Bearing)

Measurements are taken to the nearest visual landmark:

$$\mathbf{z}_k = h(\mathbf{x}_k, l) = \begin{bmatrix} \sqrt{(l_x - x_k)^2 + (l_y - y_k)^2}\\\\ \arctan\dfrac{l_y - y_k}{l_x - x_k} - \theta_k \end{bmatrix} = \begin{bmatrix} \sqrt{\Delta x_k^2 + \Delta y_k^2} = d_k\\\\ \arctan \dfrac{\Delta y}{\Delta x} - \theta_k \end{bmatrix}$$

### 3.7 Jacobian of Measurement Model (2×4)

$$\mathbf{H}_k = \frac{\partial h}{\partial \mathbf{x}} \bigg|_{\mathbf{x}_k^-}$$

$$\mathbf{H}_k = \begin{bmatrix} -\frac{\Delta x_k^-}{d_k^-} & -\frac{\Delta y_k^-}{d_k^-} & 0 & 0\\\\ \frac{\Delta y_k^-}{d_k^{-2}} & -\frac{\Delta x_k^-}{d_k^{-2}} & -1 & 0 \end{bmatrix}$$

### 3.8 Merged Measurement Model

When multiple independent sensors observe the state, their measurement models can be stacked into a single observation before the update step. This allows the EKF to process all measurements at once, preserving cross-correlation information.

**Stacked measurement vector (4×1):**
$$ \mathbf{z}_{\text{merged}} = \begin{bmatrix} \mathbf{z}_{\text{lidar}}\\\\ \mathbf{z}_{\text{imu}} \end{bmatrix} = \begin{bmatrix} \text{range}\\\\ \text{bearing}\\\\ \theta_{\text{measured}}\\\\ v_{\text{measured}} \end{bmatrix} $$

**Merged measurement function (4×1):**
$$ h_{\text{merged}}(\mathbf{x}_k) = \begin{bmatrix} h_{\text{lidar}}(\mathbf{x}_k)\\\\ h_{\text{imu}}(\mathbf{x}_k) \end{bmatrix} $$

**Merged Jacobian (4×4):**
$$ \mathbf{H}_{\text{merged}} = \begin{bmatrix} \mathbf{H}_{\text{lidar}}\\\\ \mathbf{H}_{\text{imu}} \end{bmatrix} = \begin{bmatrix} -\frac{\Delta x_k^-}{d_k^-} & -\frac{\Delta y_k^-}{d_k^-} & 0 & 0\\\\ \frac{\Delta y_k^-}{d_k^{-2}} & -\frac{\Delta x_k^-}{d_k^{-2}} & -1 & 0\\\\ 0 & 0 & 1 & 0\\\\ 0 & 0 & 0 & 1 \end{bmatrix} $$

**Merged measurement noise covariance (4×4 block-diagonal):**
$$ \mathbf{R}_{\text{merged}} = \begin{bmatrix} \mathbf{R}_{\text{lidar}} & \mathbf{0}_{2\times2}\\\\ \mathbf{0}_{2\times2} & \mathbf{R}_{\text{imu}} \end{bmatrix} $$

This merged formulation is equivalent to processing each sensor sequentially (since the sensors are independent, the order does not matter). The standard update step (section 3.11) then uses $\mathbf{H}_{\text{merged}}$, $\mathbf{R}_{\text{merged}}$, and $\mathbf{z}_{\text{merged}}$ in place of their single-sensor counterparts.

### 3.9 IMU Measurement Update (Superimposed on Odometry)

The IMU provides direct observations of **heading** (via gyroscope) and **forward velocity** (via accelerometer integration), superimposed on the odometry-based prediction:

$$ \mathbf{H}_{\text{imu}} = \begin{bmatrix} 0 & 0 & 1 & 0\\\\ 0 & 0 & 0 & 1 \end{bmatrix} $$

$$ \mathbf{z}_{\text{imu}} = \begin{bmatrix} \theta_{\text{measured}}\\\\ v_{\text{measured}} \end{bmatrix} $$

$$ \mathbf{R}_{\text{imu}} = \sigma_{\text{imu}} \mathbf{I}_2 $$

The `imuUpdate()` method follows the standard EKF measurement update, correcting both heading and velocity estimates from IMU data. This runs on top of the odometry `predict()` — the effects are superimposed, not replaced.

| Symbol | Dim | Description |
|--------|-----|-------------|
| $\theta_m$ | 1×1 | Heading measurement (true θ + noise ∝ $\sigma_{\text{imu}}$) |
| $v_m$ | 1×1 | Velocity measurement (true v + noise ∝ $\sigma_{\text{imu}}$) |
| $\sigma_{\text{imu}}$ | 1×1 | IMU noise std dev (∝ 1/accuracy) |
| $\mathbf{R}_{\text{imu}}$ | 2×2 diagonal | IMU measurement noise covariance |

### 3.10 Covariance Matrices

| Matrix | Dimension | Description | Default | Slider Mapping |
|--------|-----------|-------------|---------|----------------|
| `Q` | 4×4 diagonal | Odometry process noise | diag(0.5, 0.5, 0.1, 0.05) | Wheel accuracy 0–100 → Q = default × 1/acc |
| `Q_imu` | 4×4 diagonal | IMU process noise | diag(0.5, 0.5, 0.1, 0.05) | IMU accuracy 0–100 → Q_imu = default × 1/acc |
| `R` | 2×2 diagonal | Measurement noise (LIDAR) | diag(0.5, 0.5) | LIDAR accuracy 0–100 → R = default × 1/acc |
| `P` | 4×4 symmetric | State covariance (evolves) | 0.1 × I₄ | — |

### 3.11 Update Step

$$\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^\mathsf{T} \left( \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^\mathsf{T} + \mathbf{R} \right)^{-1}$$

$$\mathbf{x}_k^+ = \mathbf{x}_k^- + \mathbf{K}_k (\mathbf{z}_k - h(\mathbf{x}_k^-))$$

$$\mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^-$$

---

## 4. Simulation Mechanics

### 4.1 Trajectory

The reference path is a cubic Bézier curve from the starting point to the end point. Users can drag the A/B markers on the canvas to reposition. The trajectory is pre-computed at integer timesteps and interpolated with linear interpolation between steps.

Each trajectory point stores `{x, y, theta, v}` where `v = agvSpeed` (configurable via slider, range 0.5–5.0).

400 timesteps at ~60 fps ≈ 6.7 seconds per run at default speed.

### 4.2 Speed Scaling

`simTime` advances by `DT × (agvSpeed / 2.0)` per frame. At speed 5.0, the simulation completes 2.5× faster (≈2.7s). The EKF predict still runs once per frame, so higher speed means less computation per unit path length — making the corridor genuinely harder to navigate at speed.

A coordinate display below the AGV speed slider shows the current A start / B end positions (e.g., `(150, 400) → (700, 160)`), updated when route points are dragged on the canvas.

### 4.3 The Corridor Trap

| Aspect | Implementation |
|--------|---------------|
| **Timing** | Default `slipStartTime` = 120, `slipEndTime` = 280 (timesteps 120–280 of 400) |
| **LIDAR landmark dropout** | `getVisibleLandmarks()` returns empty array inside corridor |
| **GPS dropout** | GPS update gated by `!isInCorridor(simTime)` |
| **Beacon dropout** | External sensor position updates gated by `!isInCorridor(simTime)` |
| **IMU availability** | Interior sensor — always available, even inside corridor (replaces odometry predict with IMU dead-reckoning) |
| **Control corruption** | `getCorruptedControl()` subtracts a sinusoidal steering bias (max 0.015 rad/timestep) from the nominal `thetaDot` during the corridor window |
| **Noise** | Small random noise (±0.002 rad/timestep) added to prevent exact reproducibility |

### 4.4 Slip Mode

- **Deterministic**: Slip always occurs at timesteps 120–280 (fixed).
- **Semi-Random**: `computeSlipParams()` randomizes the onset (±60 steps) and offset (±40 steps) of the corridor window. Called at deploy time.

### 4.5 Collision Detection

```javascript
function checkCollision() {
  // Simple AABB overlap test with ROBOT_RADIUS margin
  // EKF estimate (x, y) vs. each rack bounding box
}
```

Racks are placed along the corridor exit (timesteps ~250–350) where EKF divergence is maximal after the landmark-free zone.

### 4.6 Divergence Check

```javascript
function checkDivergence() {
  const allow = document.getElementById('allowDivergence');
  if (allow && !allow.checked) return false;  // user opted out
  return currentDivergence > 60;              // 60px threshold
}
```

`currentDivergence = sqrt((ekf.x - trueSt.x)² + (ekf.y - trueSt.y)²)` computed every frame. The check is active throughout the simulation (no grace period), but can be disabled via the "Stop on divergence" toggle.

---

## 5. Hardware Savings Calculator

```
perfFactor = 0.40 * max(0, 1 - σ / σ_max)

where:
  σ         = sqrt(tr(P[0:2, 0:2]))  — position standard deviation from EKF
  σ_max     = 3.0                     — threshold where savings = 0
  0.40      = 40% maximum savings from EKF tuning

BOM cost = Σ enabled sensor costs:
  wheel  = $200
  lidar  = $5,000
  gps    = $800
  imu    = $3,000
  beacon = $150 each

dollar_savings = round(perfFactor × BOM_cost)
savings_pct    = dollar_savings / BOM_cost × 100
```

The dollar savings scale with the actual sensor configuration: more/better sensors increase the BOM baseline, so the same EKF quality yields larger absolute savings. Removing sensors reduces both BOM and potential savings.

---

## 6. Rendering Pipeline

### 6.1 Layer Order (back to front)

```
1. Floor + grid
2. Trajectory path (faded)
3. Racks (with hazard stripes)
4. Landmarks (beacons with active/inactive highlighting)
5. Start/End zones (A/B markers)
6. True position trail (ghost grey polyline, up to 2000 points)
7. EKF estimate trail (blue polyline)
8. Corridor zone warning overlay
9. Raw sensor measurement (red dot + landmark line)
10. True AGV (ghost, 25% opacity)
11. EKF AGV (solid blue, direction indicator with LIDAR sweep arc)
    - GPS active indicator (green dot + "GPS" label) above EKF when GPS on
    - IMU active indicator (green dot + "IMU" label) above EKF when IMU on
12. Red flash overlay (on collision/divergence)
```

---

## 7. File Architecture

| File | Responsibility |
|------|---------------|
| `index.html` | DOM structure, CDN script tags, 3-tier accordion with KaTeX, sensor/route/metrics/savings HTML |
| `styles.css` | All styles (~710 lines): layout grid, dashboard cards, accordion, sliders, responsive breakpoints, animations |
| `script.js` | `EKF` class (including `imuPredict`, `gpsUpdate`), simulation world, p5.js setup/draw, rendering (GPS/IMU indicators), UI/dashboard, sensor/business logic, Node.js exports |
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
| Click "RUN SIMULATION" | `running` flips to `true`, button text updates, simTime advances |
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
| Slider defaults | All accuracy sliders at 1.0, wheel/lidar on, GPS/IMU off, LIDAR range 400, speed 2.0 |
| Mode selector | Deterministic mode is active by default |
| Start button | Enabled, labeled "▶ RUN SIMULATION" |
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
