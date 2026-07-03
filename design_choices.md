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
- The accuracy slider scales the noise covariance matrix (Q for odometry, R for LIDAR, GPS position noise).

### Result Panel (not canvas overlay)
*Game-over and success messages appear in a card below the map, not as an overlay on top of it.*

- **Why:** Preserves full visibility of the simulation history (trails, final AGV position). The canvas is never obscured.
- Distinguishes between **divergence** (EKF >60px from true) and **collision** (EKF hit a rack) with separate messages.

### Stop on Divergence Toggle
*Users can disable the divergence check and let the simulation run regardless of EKF error.*

- **Why:** Allows observation of extreme divergence behavior without early termination. Useful for educational exploration and debugging.

---

## Sensor Architecture

### Interior vs. Exterior Sensors
Sensors are grouped into two expandable sections:

| Section | Sensors | Measurement Type | Corridor Behavior |
|---------|---------|-----------------|-------------------|
| **Interior** | Wheel Odometry | Process noise Q scaling | Always active (corrupted control) |
| **Exterior** | LIDAR, GPS, Beacons | Range-bearing or position updates | All disabled in corridor |

### Corridor Measurement Dropout
Inside the featureless corridor, ALL exterior measurements stop:
- **LIDAR** — `getVisibleLandmarks()` returns `[]`
- **GPS** — gated by `!isInCorridor(simTime)`
- **Beacons** — their position-update loop is gated by `!isInCorridor(simTime)` (and also by LIDAR being enabled)
- Only **wheel odometry** + corrupted control remains

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
4. **Active Beacons** — LiDAR beacons within LIDAR range glow green with "ACTIVE" label.

### Collision Feedback (Overcooked-style)
- On crash: screen shake (CSS `translate` jitter for ~500ms), red flash on canvas, result panel below showing **Collision** or **Divergence** message with "TUNE AGAIN" button.
- On success: green result panel with achievement-style metrics below the canvas.

---

## Layout Architecture

```
Desktop (≥900px):
┌────────────────────────────────┬──────────────┐
│   Left Column                  │ Right Column │
│  ┌─────────────────┐          │  Dashboard    │
│  │   p5.js Canvas   │          │  - DEPLOY btn │
│  ├─────────────────┤          │  - Speed      │
│  │   Result Panel   │          │  - Stop/Diver │
│  ├─────────────────┤          │  - Sensors    │
│  │ Sensor Explains  │          │  - Route      │
│  ├─────────────────┤          │  - Metrics    │
│  │ EKF Accordion   │          │  - Savings   │
│  │ - Executive     │          │  - Business   │
│  │ - Engineer      │          └──────────────┘
│  │ - Specialist    │
│  └─────────────────┘
```

- **Left column (3fr):** canvas → result panel → sensor explanations → EKF accordion (stacked vertically)
- **Right column (2fr):** dashboard with all controls, metrics, and business panels
- Responsive: single column stack on mobile (<900px)

---

## Gamification Mechanics

| Mechanic | Implementation |
|----------|---------------|
| **Sensor toggles** | Enable/disable odometry, LIDAR, GPS independently |
| **Accuracy sliders** | Per-sensor accuracy (0.1–5.0) scales noise covariances |
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
- Sensor BOM cost: sum of enabled sensor costs (`wheel=$200`, `lidar=$5000`, `gps=$800`, `each beacon=$150`)
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

The accordion is placed **below the map and sensor descriptions** in the left column, forming a natural reading flow: watch the simulation → see the result → understand the sensors → learn the math.
