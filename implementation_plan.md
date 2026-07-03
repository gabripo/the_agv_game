# Implementation Plan — "The EKF Tuner: Navigating Uncertainty"

## File Structure

```
the_agv_game/
├── index.html              # Single-page app HTML + CDN script tags
├── styles.css              # All styles (layout, cards, accordion, sliders, responsive)
├── script.js               # All JS: EKF core + p5.js simulation + dashboard logic
├── design_choices.md       # Design decisions document
├── docs/architecture.md    # Technical architecture deep-dive
├── implementation_plan.md  # This file
├── README.md               # Project readme
├── tests/                  # Unit and integration tests
├── .github/workflows/      # CI/CD pipelines
└── (Dockerfile, nginx.conf, etc.)
```

---

## Build Order (16 Steps)

### Phase 1: EKF Math Engine

#### Step 1 — Scaffold `index.html`
- Add CDN script tags for: `p5.js`, `math.js`, `KaTeX` CSS + JS
- Empty `<div id="app">` container for the full layout
- Link `script.js`
- CSS reset + base typography

#### Step 2 — `EKFSlam` class in `script.js`
- **State vector** (4×1): `[x, y, θ, v]ᵀ`
- **Control input** (2×1): `[v̇, θ̇]ᵀ`
- **Covariance matrix P** (4×4): uncertainty around state estimate
- **Process noise Q** (4×4): diagonal, user-tunable
- **Measurement noise R** (2×2): diagonal, user-tunable

**Methods:**
- `predict(control, dt)` — non-linear motion model + covariance propagation via Jacobian
- `jacobianF(state, control, dt)` — 4×4 analytic Jacobian of motion model
- `predictMeasurement(landmark)` — range + bearing from state to landmark
- `jacobianH(state, landmark)` — 2×4 analytic Jacobian of measurement model
- `update(measurement, landmark)` — standard EKF update equations
- `gpsUpdate(measurement)` — direct position update (4-element [x, y, σx, σy])
- `imuUpdate(thetaMeasured, noiseTheta)` — direct heading observation (H = [0,0,1,0])
- `setTuning(Qscale, Rscale)` — scale diagonal entries of Q and R

#### Step 3 — Console unit-test
- Run 50 open-loop prediction steps, verify:
  - Matrix dimensions remain correct at every step
  - `trace(P)` monotonically increases during predict-only (no measurements)
  - `trace(P)` decreases after an update step
  - Kalman Gain matrix K converges to steady-state with constant Q/R

---

### Phase 2: p5.js Simulation

#### Step 4 — Warehouse background (`setup()`)
- Floor: dark grey with subtle grid lines
- Racks: arranged rectangular obstacles with collision bounding boxes
- Start zone (A): green highlight at `(100, 500)`
- End zone (B): blue highlight at `(700, 100)`
- Featureless corridor zone: a diagonal band between `x ∈ [350, 550]` with visual indicator ("⚠ LOW FEATURES")

#### Step 5 — True path + slip injection (`draw()`)
- Precompute reference trajectory as array of `{x, y, theta, v}` waypoints
- At each frame, interpolate true position from trajectory
- **During corridor zone:**
  - Inject `slip_x`, `slip_y` perturbation to true position
  - Apply random yaw drift (`delta_theta += random(-0.05, 0.05)`)
  - Control input sent to EKF does NOT include this perturbation
  - In semi-random mode: slip onset time is randomized within corridor bounds

#### Step 6 — Sensor simulation
- Define 8–10 fixed landmark positions across the map
- At each timestep:
  - Compute true range & bearing to nearest landmark
  - Add Gaussian noise scaled by R diagonal
  - **Outside corridor:** pass measurement to EKF update
  - **Inside corridor:** skip measurement (simulate featureless environment)
  - **GPS:** if enabled + outside corridor, generate noisy [x, y, σx, σy] → `EKF.gpsUpdate()`
  - **IMU:** if enabled, generate noisy heading → `EKF.imuUpdate()` (always available, interior)
- Render raw measurements as red dots at `(ekf.state.x, ekf.state.y) + noisy_range_bearing`
- Render GPS active indicator (green dot + "GPS") and IMU active indicator (green dot + "IMU") on EKF estimate

#### Step 7 — Wire EKF into draw loop
```js
function draw() {
  if (!running) return;

  trueState = getTrueState(t);

  // Inject slip if in corridor
  if (inCorridor(t)) {
    slipPerturbation = computeSlip(t);
    applySlip(trueState, slipPerturbation);
  }

  // EKF predict (using control input, NOT slip-perturbed)
  control = { v_dot: 0.5, theta_dot: 0.1 };
  ekf.predict(control, dt);

  // Sensor measurement
  landmark = getNearestLandmark(trueState);
  if (landmark && !inCorridor(t)) {
    measurement = noisyRangeBearing(trueState, landmark);
    ekf.update(measurement, landmark);
  }

  // GPS update (exterior, disabled in corridor)
  if (gpsEnabled && !inCorridor(t)) {
    noisyGPS = getNoisyGps(trueState, gpsAccuracy);
    ekf.gpsUpdate(noisyGPS);
  }

  // IMU heading update (interior, always available)
  if (imuEnabled) {
    noisyHeading = trueState.theta + headingNoise;
    ekf.imuUpdate(noisyHeading, noiseVariance);
  }

  // Collision check
  if (checkCollision(ekf.state, racks)) {
    gameOver();
  }

  // Render
  drawTrueState(trueState);
  drawMeasurement(measurement, landmark);
  drawEKFEstimate(ekf.state);
  drawTrailingPath(ekf.history);
}
```

#### Step 8 — Collision + Game Over / Success
- **Rack bounding boxes:** 6–8 rectangles placed along path edges
- **Collision:** EKF estimate bounding box intersects a rack → `gameOver()`
- **Screen shake:** CSS `animation` on canvas container for 500ms
- **Red flash:** Semi-transparent red overlay, fading over 1s
- **Success:** EKF reaches within 50px of Point B → "ROUTE COMPLETE" overlay
- Both states: show "TUNE AGAIN" button that resets to slider config

---

### Phase 3: UI & Dashboard

#### Step 9 — Sensor toggles & accuracy sliders
- Interior: Wheel Odometry (on by default), IMU (off by default)
- Exterior: LIDAR (on by default), GPS (off by default), Beacons (5 default)
- Each sensor has:
  - Enable/disable toggle
  - Accuracy slider (0–100, default 1.0) — maps to inverse noise covariance (σ ∝ 1/accuracy)
  - Dynamic red→yellow→green gradient background fill proportional to value
  - Tooltip and `ⓘ` info button explaining the accuracy→covariance mapping
- LIDAR also has a range slider (100–600px, default 400)
- Each beacon has its own accuracy slider

#### Step 10 — Slip mode selector
- Radio group: `[Deterministic] [Semi-Random]`
- Styled as pill buttons, active one highlighted with accent color
- Locked after START is pressed

#### Step 11 — START button
- Label: "▶ RUN SIMULATION"
- Also: "Reset Configuration" button to restore all defaults
- On click:
  - Lock sliders + mode selector (visual lock overlay)
  - Set `running = true`
  - Call `loop()` to begin p5.js draw cycle
- After game over or success: button relabels to "TUNE AGAIN" and re-enables sliders

#### Step 12 — Live metrics panel
- **Status indicator:** LED-style dot: green = OK, yellow = corridor (warning), red = diverging
  - Description text explaining Status, Uncertainty, and Divergence fields
- **Position Uncertainty:** `trace(P[0:2, 0:2]).toFixed(4)` — updated every frame
- **Divergence:** current EKF position error in pixels
- **Sensor Cost Savings:** percentage + progress bar

#### Step 13 — Hardware Savings Calculator
- Formula: `savings = 0.40 × max(0, 1 − trace(P[0:2,0:2]) / σ_max)`
- BOM formula (KaTeX): `BOM = C_wheel + C_lidar + C_gps + C_imu + n_beacon × C_beacon`
- Formula spoiler: Sensor costs listed (wheel $200, lidar $5,000, gps $800, imu $3,000, beacon $150 ea.)
- Display: percentage number + a horizontal gradient bar (red → yellow → green)
- Dynamic text beneath: *"At this tuning level, each AGV saves ~$X in sensor specification costs, reducing BOM by Y%."*
- Savings scale with actual sensor configuration

#### Step 14 — Explanatory accordion (3 tiers) + spoilers
- All accordions start **collapsed** using `<details>` / `<summary>` HTML elements
- **Executive:** Short paragraph with dark-room analogy
- **Engineer:** Predict-Update loop explanation, covariance matrices explained as confidence scores
- **Specialist:** KaTeX-rendered equations:
  - `K_k = P_k^- H_k^T (H_k P_k^- H_k^T + R_k)^{-1}`
  - `x_k^+ = x_k^- + K_k (z_k - h(x_k^-))`
  - Jacobian definition: `F_k = ∂f/∂x |_{x_{k-1}, u_k}`
  - Nested "Symbol legend" spoiler with KaTeX definitions for each variable

---

### Phase 4: Polish

#### Step 15 — Responsive CSS
- `@media (min-width: 900px)`: 2-column grid
- `@media (max-width: 899px)`: single column, stacked
- Smooth transitions on slider movement, accordion open/close
- Hover states on all interactive elements

#### Step 16 — Integration test
- **Scenario A:** Q = 0.1, R = 5.0 (trust odometry heavily) → should crash in corridor
- **Scenario B:** Q = 5.0, R = 0.1 (trust sensors heavily) → should crash in corridor (no landmarks)
- **Scenario C:** Q = 1.0, R = 1.0 (balanced) → should successfully traverse corridor
- Verify all three produce expected outcomes

---

## Matrix Dimensions Reference

| Symbol | Dim | Description |
|--------|-----|-------------|
| `x` | 4×1 | State vector `[x, y, θ, v]ᵀ` |
| `u` | 2×1 | Control input `[v̇, θ̇]ᵀ` |
| `P` | 4×4 | State covariance |
| `F` | 4×4 | Jacobian of motion model |
| `Q` | 4×4 | Process noise covariance (diagonal) |
| `H` | 2×4 | Jacobian of measurement model (range-bearing) |
| `R` | 2×2 | Measurement noise covariance (diagonal) |
| `H_imu` | 1×4 | IMU Jacobian `[0, 0, 1, 0]` (selects θ) |
| `R_imu` | 1×1 | IMU heading noise variance |
| `K` | 4×2 | Kalman Gain |
| `S` | 2×2 | Innovation covariance |
| `z` | 2×1 | Measurement vector `[range, bearing]ᵀ` |
| `I` | 4×4 | Identity matrix |

---

## State Machine

```
IDLE ──(tune sliders, select mode, place beacons, configure route)──→ READY
                                                                        │
                                                                  (press DEPLOY)
                                                                        │
                                                                        ▼
                                                                    RUNNING
                                                                        │
                                                          ┌─────────────┼──────────────┐
                                                          │             │              │
                                                          ▼             ▼              ▼
                                                   CORRIDOR_ENTRY  CORRIDOR_EXIT   DIVERGENCE
                                                   (no landmarks,   (landmarks     │
                                                    slip + corrupt   return)       │
                                                    control, IMU     │              │
                                                    still active)    │              │
                                                          │             │              │
                                                          └──────┬──────┘              │
                                                                 │                     │
                                                                 ▼                     ▼
                                                          (recovery)             GAME_OVER
                                                                 │                     │
                                                                 ▼                     │
                                                          SUCCESS                    │
                                                                 │                     │
                                                                 └──────┬──────────────┘
                                                                        │
                                                                  (press TUNE AGAIN
                                                                   or RESET CONFIG)
                                                                        │
                                                                        ▼
                                                                      IDLE
```
