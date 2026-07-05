# Design Choices — "The EKF Tuner: Navigating Uncertainty"

## Project Identity
A single-page portfolio piece demonstrating deep understanding of Extended Kalman Filters (EKF) for vehicle state estimation, packaged as an interactive simulation with a business-value dashboard.

---

## Frontend Stack

| Choice | Reason |
|--------|--------|
| **Vanilla HTML + CSS + JS** | Zero build step, single-file deploy, focus on concepts over tooling |
| **p5.js (CDN)** | Mature 2D canvas library; `setup()`/`draw()` loop maps perfectly to real-time simulation |
| **math.js (CDN)** | Clean JavaScript matrix operations (multiply, inverse, transpose, diag) — avoids hand-rolling linear algebra |
| **KaTeX (CDN)** | Renders LaTeX equations to crisp SVG at runtime; lightweight (~40KB gzipped), no MathJax overhead |

---

## User Interaction Decisions

### Start Button (not auto-run)
*User tunes sensors first, then presses "RUN SIMULATION" to start.*

- **Why:** Forces deliberate configuration before the run. Mimics real-world workflow: configure → deploy → observe.
- **Portfolio narrative:** "Demonstrates that I understand state estimation tuning happens *before* deployment, not reactively."

### Slip Mode Selector (Deterministic or Semi-Random)
*User chooses the slip behavior before starting.*

- **Deterministic:** Slip always occurs at the same time & location. Good for demos and repeatable testing.
- **Semi-Random:** Slip occurs at a random position within the corridor zone. Rewards robust tuning, adds replayability.
- **Portfolio narrative:** "Shows awareness that real SLAM environments combine predictable failure modes (known featureless corridors) with stochastic disturbances."

### Sensor Accuracy vs Toggle
*Each sensor has both an enable toggle and an accuracy slider.*

- **Why:** Decouples "do I have this sensor?" from "how good is it?" — users can experiment with having a cheap/bad GPS vs. an expensive/accurate one, or remove a sensor entirely and rely on remaining sensors.
- The accuracy slider (0–100 range) maps to inverse noise covariance: noise σ ∝ 1/accuracy. At 0, noise → ∞ (sensor ignored); at 100, noise → 0 (perfect measurement). Each slider has a dynamic red→yellow→green background fill proportional to its value.
- Tooltips on each slider and an `ⓘ` info button explain the accuracy→covariance mapping in plain language.

### Reset Configuration Button
*One-click restore of all slider, toggle, and route defaults.*

- **Why:** Allows users to quickly recover from extreme tuning experiments without manually resetting each control. Restores sensor toggles, accuracy sliders (to 1.0), speed (2.0), stop-on-divergence (on), A/B route points, slip mode (deterministic), and default beacons.

### Result Panel (not canvas overlay)
*Game-over and success messages appear in a card below the map, not as an overlay on top of it.*

- **Why:** Preserves full visibility of the simulation history (trails, final AGV position). The canvas is never obscured.
- Distinguishes between **divergence** (EKF >60px from true) and **collision** (EKF hit a rack) with separate messages.

### Stop on Divergence Toggle
*Users can disable the divergence check and let the simulation run regardless of EKF error.*

- **Why:** Allows observation of extreme divergence behavior without early termination. Useful for educational exploration and debugging.

### Live Metrics Description
*The Live Metrics section includes explanatory text clarifying each field.*

- **Status:** Shows the current simulation state (OK, Corridor, Diverging, Completed, Crashed).
- **Uncertainty:** `trace(P[0:2, 0:2])` — the sum of position variances from the EKF covariance matrix. Grows without measurements, shrinks on updates.
- **Divergence:** Euclidean distance between EKF estimate and true position. Threshold for divergence check is 60px.

---

## Sensor Architecture

### Interior vs. Exterior Sensors
Sensors are grouped into two expandable sections:

| Section | Sensors | Measurement Type | Corridor Behavior |
|---------|---------|-----------------|-------------------|
| **Interior** | Wheel Odometry, IMU | Process noise Q scaling, θ + v measurement update | Always active (odometry: corrupted control; IMU: superimposed θ/v correction) |
| **Exterior** | LIDAR, GPS, Beacons | Range-bearing or position updates | All disabled in corridor |

### Corridor Measurement Dropout
Inside the featureless corridor, ALL exterior measurements stop:
- **LIDAR** — `getVisibleLandmarks()` returns `[]`
- **GPS** — gated by `!isInCorridor(simTime)`
- **Beacons** — their position-update loop is gated by `!isInCorridor(simTime)` (and also by LIDAR being enabled)
- The **IMU** provides heading ($\theta$) and velocity ($v$) measurements via `imuUpdate()`, superimposed on the odometry-based prediction (both contribute simultaneously, always available)
- Only **wheel odometry** (corrupted control) + **IMU** ($\theta$/$v$ correction) remains

---

## Visual Design

### Color Palette

| Token | Hex | Usage |
|-------|-----|-------|
| Primary (dark) | `#1a2634` | Page background, nav |
| Surface (slate) | `#2c3e50` | Cards, panels |
| Accent (blue) | `#3498db` | Buttons, active states, EKF estimate, LIDAR range |
| Success (green) | `#2ecc71` | Optimal tuning indicator, savings bar, active beacon |
| Danger (red) | `#e74c3c` | Crash state, warning indicators |
| Warning (orange) | `#f39c12` | Corridor zone markers |
| Text | `#ecf0f1` | Primary text on dark surfaces |
| Text-secondary | `#95a5a6` | Secondary labels, hints |

### Canvas Render Layers
1. **True Position** — pale/ghost grey at 25% opacity. The "ground truth" the user never sees directly.
2. **Raw Sensor Measurements** — small red dots jumping around the true position. Visually noisy.
3. **EKF Estimated Position** — solid blue-grey rectangle with a trailing path polyline. The smoothed, filtered state.
4. **GPS Indicator** — When GPS is enabled and active (outside corridor), a green dot with "GPS" label above the EKF estimate.
5. **IMU Indicator** — When IMU is enabled, a green dot with "IMU" label above the EKF estimate (always active since IMU is interior).
6. **Active Beacons** — LIDAR beacons within LIDAR range glow green with "ACTIVE" label.

### Collision Feedback (Overcooked-style)
- On crash: screen shake (CSS `translate` jitter for ~500ms), red flash on canvas, result panel below showing **Collision** or **Divergence** message with "TUNE AGAIN" button.
- On success: green result panel with achievement-style metrics below the canvas.

---

## Layout Architecture

```
Desktop (≥900px):
┌────────────────────────────────┬────────────────┐
│   Left Column                  │ Right Column   │
│  ┌─────────────────┐          │  Dashboard      │
│  │   p5.js Canvas   │          │  - DEPLOY btn   │
│  ├─────────────────┤          │  - Speed + Coords│
│  │   Result Panel   │          │  - Stop/Diver   │
│  ├─────────────────┤          │  ▼ Sensors (col)│
│  │   EKF Accordion │          │    Interior     │
│  │  - Executive    │          │    Exterior     │
│  │  - Engineer     │          │  ▼ Route (coll) │
│  │  - Specialist   │          │  ▼ Live Metrics │
│  │    + symbol leg │          │  ▼ Savings Calc │
│  └─────────────────┘          │  ▼ Business Imp │
│                                └────────────────┘
```

- **Left column (3fr):** canvas → result panel → EKF accordion (stacked vertically)
- **Right column (2fr):** dashboard with all controls, metrics, and business panels
- All accordion sections (Sensors, Route, Metrics, Savings, Business) start **collapsed** — user opens what they need
- Dashboard content flows naturally (no `max-height` / scroll clipping)
- Sensor descriptions moved from standalone card to inline slider tooltips and `ⓘ` info hints
- Responsive: single column stack on mobile (<900px)

---

## Gamification Mechanics

| Mechanic | Implementation |
|----------|---------------|
| **Sensor toggles** | Enable/disable odometry, LIDAR, GPS, IMU independently |
| **Accuracy sliders** | Per-sensor accuracy (0–100) maps to inverse noise covariance (σ ∝ 1/accuracy) |
| **Dynamic slider backgrounds** | Red→yellow→green gradient fill proportional to slider value |
| **Reset Configuration** | One-click restore of all defaults |
| **Coordinate display** | Live A/B route point readout below speed slider |
| **GPS indicator** | Green dot + "GPS" label on EKF estimate when GPS active |
| **IMU indicator** | Green dot + "IMU" label on EKF estimate when IMU active |
| **LIDAR range slider** | Controls detection radius (100–600px) |
| **AGV speed slider** | Scales simTime advance; faster = corridor is traversed in fewer real frames |
| **The trap** | Mid-route slip event + complete exterior measurement dropout in corridor |
| **Stop on divergence** | Optional 60px threshold check (toggleable) |
| **Failure states** | Two distinct outcomes: Divergence (>60px error) or Collision (rack hit) |
| **Success state** | EKF recovers → reaches destination → success metrics |
| **Replayability** | Sensor combos, slip mode, speed, accuracy levels, beacon placement |

---

## Hardware Savings Calculator

- Every frame compute `σ = sqrt(tr(P[0:2, 0:2]))` (position standard deviation from the state covariance matrix)
- EKF performance factor: `perfFactor = 0.40 × max(0, 1 − σ / 3.0)`
- Sensor BOM cost: sum of enabled sensor costs (`wheel=$200`, `lidar=$5000`, `gps=$800`, `imu=$3000`, `each beacon=$150`)
- Dollar savings: `perfFactor × BOM_cost`
- Savings percentage: `dollar_savings / BOM_cost` displayed with progress bar
- Dynamic narrative shows BOM breakdown and dollar savings

---

## Explanatory Tiers (Accordion)

| Tier | Audience | Content |
|------|----------|---------|
| **Executive View** | C-level, non-technical | Walking-in-a-dark-room analogy; business continuity focus |
| **Engineer View** | Technical peers | Predict-Update loop explanation; Q and R as confidence dials |
| **Specialist View** | PhDs, algorithm leads | Kalman Gain equation, Jacobian linearization, rendered via KaTeX SVG |

The accordion is placed **below the map** in the left column, forming a natural reading flow: watch the simulation → see the result → learn the math.

The **Specialist** tier includes a nested "Symbol legend" spoiler with KaTeX-rendered definitions for every variable. The **Hardware Savings Calculator** similarly has a "Formula & sensor costs" spoiler showing the BOM formula and per-sensor price breakdown.
