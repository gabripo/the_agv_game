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
*User tunes sliders first, then presses "Deploy AGV" to start.*

- **Why:** Forces deliberate tuning before the run. Mimics real-world workflow: configure → deploy → observe.
- **Portfolio narrative:** "Demonstrates that I understand state estimation tuning happens *before* deployment, not reactively."

### Slip Mode Selector (Deterministic or Semi-Random)
*User chooses the slip behavior before starting.*

- **Deterministic:** Slip always occurs at the same time & location. Good for demos and repeatable testing.
- **Semi-Random:** Slip occurs at a random position within the corridor zone. Rewards robust tuning, adds replayability.
- **Portfolio narrative:** "Shows awareness that real SLAM environments combine predictable failure modes (known featureless corridors) with stochastic disturbances."

---

## Visual Design

### Color Palette

| Token | Hex | Usage |
|-------|-----|-------|
| Primary (dark) | `#1a2634` | Page background, nav |
| Surface (slate) | `#2c3e50` | Cards, panels |
| Accent (blue) | `#3498db` | Buttons, active states, EKF estimate |
| Success (green) | `#2ecc71` | Optimal tuning indicator, savings bar |
| Danger (red) | `#e74c3c` | Crash state, warning indicators |
| Text | `#ecf0f1` | Primary text on dark surfaces |
| Text-secondary | `#95a5a6` | Secondary labels, hints |

### Canvas Render Layers
1. **True Position** — pale/ghost grey at 25% opacity. The "ground truth" the user never sees directly.
2. **Raw Sensor Measurements** — small red dots jumping around the true position. Visually noisy.
3. **EKF Estimated Position** — solid blue-grey rectangle with a trailing path polyline. The smoothed, filtered state.

### Collision Feedback (Overcooked-style)
- On crash: screen shake (CSS `translate` jitter for ~500ms), red flash overlay, large "AGV MALFUNCTION" text
- On success: green glow, "ROUTE COMPLETE" with achievement-style metrics

---

## Layout Architecture

```
Desktop (≥900px):                    Mobile (<900px):
┌─────────────────┬──────────┐       ┌─────────────────┐
│   p5.js Canvas  │ Dashboard│       │   p5.js Canvas  │
│   (left, flex)  │ (right)  │       │                 │
├─────────────────┴──────────┤       ├─────────────────┤
│   Explanatory Accordion     │       │   Dashboard     │
│   (full width, below)      │       ├─────────────────┤
└────────────────────────────┘       │   Accordion     │
                                     └─────────────────┘
```

- CSS Grid for desktop layout (2 columns: 3fr canvas + 2fr dashboard)
- Single column stacking on mobile
- All cards use `border-radius: 12px`, `box-shadow: 0 4px 6px rgba(0,0,0,0.3)`

---

## Gamification Mechanics

| Mechanic | Implementation |
|----------|---------------|
| **Tuning sliders** | Q (Process Noise) and R (Measurement Noise) — user adjusts before run |
| **The trap** | Mid-route slip event + landmark dropout in corridor zone |
| **Failure state** | EKF estimate diverges → collides with rack → Game Over |
| **Success state** | EKF recovers → reaches Point B → success screen + cost savings metric |
| **Replayability** | Slip mode (deterministic vs semi-random) + slider experimentation |

---

## Hardware Savings Calculator

- Every frame compute `σ = trace(P[0:2, 0:2])` (position covariance from the state covariance matrix)
- Map to savings via: `savings = 0.40 × max(0, 1 − σ / σ_max)`
- Display as percentage with a green progress bar
- Dynamic text: "Each unit saves ~$X in lidar spec costs at this tuning level"

---

## Explanatory Tiers (Accordion)

| Tier | Audience | Content |
|------|----------|---------|
| **Executive View** | C-level, non-technical | Walking-in-a-dark-room analogy; business continuity focus |
| **Engineer View** | Technical peers | Predict-Update loop explanation; Q and R as confidence dials |
| **Specialist View** | PhDs, algorithm leads | Kalman Gain equation, Jacobian linearization, rendered via KaTeX SVG |
