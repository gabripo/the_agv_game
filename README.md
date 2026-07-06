# The EKF Tuner — Navigating Uncertainty

> An interactive portfolio piece demonstrating Extended Kalman Filter (EKF) sensor fusion for AGV state estimation in a simulated warehouse environment.

[![Tests](https://github.com/gabripo/the_agv_game/actions/workflows/test.yml/badge.svg)](https://github.com/gabripo/the_agv_game/actions/workflows/test.yml)
[![Deploy](https://github.com/gabripo/the_agv_game/actions/workflows/deploy.yml/badge.svg)](https://github.com/gabripo/the_agv_game/actions/workflows/deploy.yml)

**Live Demo:** [https://gabripo.github.io/the_agv_game](https://gabripo.github.io/the_agv_game)

---

## Overview

<img src="docs/simulation.gif" alt="EKF Tuner simulation with default settings" style="max-width: 100%; border-radius: 8px; margin-bottom: 12px;">

This single-page web application simulates an Automated Guided Vehicle (AGV) navigating through a warehouse with a "featureless corridor" — a zone where visual landmarks disappear and wheel odometry becomes unreliable. The user tunes two EKF covariance matrices (Process Noise `Q` and Measurement Noise `R`) before deployment and observes how their tuning affects the vehicle's ability to navigate the trap.

### Key Features

- **Interactive EKF Simulation** — Real-time 2D visualization of true position, raw sensor measurements, and the EKF-estimated state
- **Sensor Configuration** — Enable/disable wheel odometry, LIDAR, GPS, and IMU independently; adjust per-sensor accuracy (0–50 sliders, mapped to inverse noise covariance); control LIDAR detection range; add/remove visual beacons on the map
- **The Trap** — A featureless corridor where landmarks vanish, GPS/LIDAR/beacon measurements drop out, and the control input is corrupted (simulating wheel slip on a slippery floor)
- **Collision Physics** — A poorly-tuned filter causes the EKF estimate to diverge into warehouse racking, triggering a Game Over with distinct collision vs. divergence messages
- **Hardware Savings Calculator** — Computes BOM savings based on EKF position covariance and the actual sensor configuration (savings = EKF quality × sensor BOM cost). BOM includes wheel ($200), lidar ($5,000), gps ($800), imu ($3,000), and beacons ($150 ea.)
- **AGV Speed Control** — Slider to adjust traversal speed; higher speed makes the corridor harder to navigate
- **Stop on Divergence** — Toggle to enable/disable the 60px divergence threshold stop
- **Three-Tier Explanatory Accordion** — Executive (business analogy), Engineer (predict-update loop), Specialist (full EKF math with KaTeX-rendered equations and symbol legend spoiler) — displayed below the map
- **Reset Configuration** — One-click restore of all defaults (sensor toggles, accuracy sliders, route points, speed, corridor mode)
- **Coordinate Display** — Live A (start) / B (end) coordinate readout below the AGV speed slider
- **GPS & IMU Indicators** — Green dots and labels above the EKF estimate on canvas when GPS or IMU are active
- **Dynamic Slider Feedback** — Slider backgrounds transition red→yellow→green proportionally; tooltips explain the accuracy→covariance mapping

### Technology Stack

| Layer | Library | Role |
|-------|---------|------|
| Rendering | [p5.js](https://p5js.org/) (CDN) | 2D canvas simulation |
| Linear Algebra | [math.js](https://mathjs.org/) (CDN) | Matrix operations for EKF |
| Math Typesetting | [KaTeX](https://katex.org/) (CDN) | SVG equation rendering |
| Layout | Vanilla CSS (Grid/Flexbox) | Responsive dashboard |
| Logic | Vanilla JavaScript | EKF core, simulation, UI |
| Testing (unit) | Mocha + Chai + math.js | EKF math & pure function tests |
| Testing (e2e) | Playwright + Firefox | Headless browser integration test |

---

## Quick Start

### In a Browser (no server needed)

Open `index.html` directly — all libraries are loaded via CDN.

### Local Dev Server

```bash
npm install
npm start
```

Opens at [http://localhost:8080](http://localhost:8080).

### Docker

```bash
docker compose up --build
```

Opens at [http://localhost:8080](http://localhost:8080).

---

## Usage

1. **Configure sensors** — Toggle wheel odometry, LIDAR, GPS, IMU, and beacons on/off. Adjust per-sensor accuracy (0–50 sliders, higher = more accurate = lower noise) and LIDAR detection range.
2. **Set AGV speed** — Slider below RUN SIMULATION controls how fast the AGV traverses the path. Higher speed = fewer frames in the corridor = harder to navigate.
3. **Select slip mode** — Deterministic (fixed timing) or Semi-Random (variable timing within corridor zone).
4. **(Optional) Toggle "Stop on divergence"** — When checked, the simulation stops if the EKF estimate drifts >60px from the true position. Uncheck to let the run continue (collision/off-canvas stops still apply).
5. **Add/remove map beacons** — Click "+ Add Beacon" then click the map to place GPS-style position beacons. Drag to reposition. Remove with the × button.
6. **Press "RUN SIMULATION"** — The simulation starts. Watch the EKF estimate (solid blue) track the true position (ghost grey) using noisy sensor measurements (red dots). Beacons within LIDAR range glow green.
7. **Observe the corridor** — Halfway through, all exterior measurements stop (LIDAR, GPS, beacons). The control input is corrupted (wheel slip). The EKF must rely on its tuning and odometry to survive.
8. **Win or crash** — A well-tuned filter recovers after the corridor. A poorly-tuned filter diverges: the simulation stops with a **Diverged** or **Collision** message below the map.
9. **Review** — The full EKF and true path remains visible. Result panel and explanatory accordion are shown below the canvas.

### Recommended Tuning Experiments

| Tuning | Expected Outcome | Why |
|--------|-----------------|-----|
| Q=0.1, R=5.0 | ❌ Crashes in corridor | Low Q = small covariance → ignores new measurements after corridor |
| Q=5.0, R=0.1 | ⚠ Survives, jittery | High Q + low R = aggressive corrections; noisy tracking |
| Q=1.0, R=1.0 | ✓ Balanced recovery | Reasonable covariance growth + measurement trust |

---

## Controls Reference

| Control | Location | Description |
|---------|----------|-------------|
| **AGV Speed** | Below RUN SIMULATION button | Traversal speed (0.5–5.0). Scales `simTime` advance rate. Higher = faster = fewer frames in corridor. |
| **Stop on divergence** | Below speed slider | When checked, stops simulation if EKF estimate >60px from true position. |
| **Wheel Odometry** | Sensors Configuration > Interior | Toggle on/off. Accuracy slider (0–50) scales process noise Q. |
| **IMU** | Sensors Configuration > Interior | Toggle on/off (off by default). Accuracy slider (0–50) scales heading measurement noise. Always available (interior sensor, works in corridor). Cost: $3,000. |
| **LIDAR** | Sensors Configuration > Exterior | Toggle on/off. Accuracy slider (0–50) scales measurement noise R. Range slider (100–600px) controls detection radius. |
| **GPS** | Sensors Configuration > Exterior | Toggle on/off. Accuracy slider (0–50) scales GPS position noise. **Disabled inside corridor.** |
| **Beacons** | Sensors Configuration > Exterior | Default 5 placed on map. Add/remove via button. Each has an accuracy slider. Provide GPS-style position updates. |
| **Reset Configuration** | Below Sensors Configuration | Restores all defaults: sensors, accuracy, speed, route, slip mode, beacons, corridor params. |
| **Coordinate Display** | Below AGV speed slider | Shows A start / B end positions (e.g., (150, 400) → (700, 160)). |
| **Slip Mode** | Route Configuration | Deterministic (fixed) or Semi-Random (variable onset/offset within corridor). |
| **Route A/B** | Route Configuration | Drag green (A) and blue (B) markers on the map to reposition start/end. |

## Project Structure

```
the_agv_game/
├── index.html                  # Single-page app (HTML + CDN script tags)
├── styles.css                  # All styles (CSS Grid/Flexbox, ~710 lines)
├── script.js                   # All JS: EKF, p5.js, UI, dashboard
├── design_choices.md           # Design decisions & rationale
├── implementation_plan.md      # Development plan & architecture
├── README.md                   # This file
├── package.json                # npm scripts & dev dependencies
├── Dockerfile                  # Nginx container for static serving
├── docker-compose.yml          # Docker Compose config
├── nginx.conf                  # Nginx configuration
├── .gitignore
├── .github/
│   └── workflows/
│       ├── deploy.yml          # GitHub Pages deployment
│       └── test.yml            # CI test runner
├── tests/
│   ├── setup.js                # Test environment (math.js global)
│   ├── test_ekf.js             # EKF class unit tests (28 tests)
│   └── test_functions.js       # Pure function tests (17 tests)
└── docs/
    └── architecture.md         # Technical architecture deep-dive
```

---

## Running Tests

### Unit Tests (45 tests)

```bash
npm test
```

- **EKF math** (28 tests) — Constructor, state accessors, normalizeAngle, motion model `f()`, Jacobian `F`, predict, measurement model `h()`, Jacobian `H`, update, setTuning, uncertainty metrics, numerical stability
- **Pure functions** (17 tests) — cubicBezier, measurementToPosition, isInCorridor, calculateSavings, getCorruptedControl, noisyMeasurement

### Headless Browser Integration Test (1 test)

```bash
npm run test:headless
```

Requires [Playwright](https://playwright.dev/) with Firefox. Spawns a headless Firefox instance, loads `index.html`, verifies:
- Canvas renders with visible content
- p5.js `setup()` and `draw()` are defined and execute
- Slider controls have expected default values
- "RUN SIMULATION" button starts the simulation
- The simulation advances time and the EKF state updates
- Route completes without JavaScript errors

### Full CI Pipeline

```bash
npm run test:all       # unit + headless
```

---

## Deployment

### GitHub Pages (CI/CD Pipeline)

Two GitHub Actions workflows enforce quality before deployment:

**`.github/workflows/test.yml`** — Runs on every PR and push to `main`/`master`:
1. **Unit Tests** — 45 Mocha tests for EKF math and pure functions
2. **Headless Browser Test** — Playwright + Firefox integration test (runs after unit tests pass)

**`.github/workflows/deploy.yml`** — Runs on push to `main`/`master`:
1. **Unit Tests** — same suite as above
2. **Headless Browser Test** — same integration test
3. **Deploy** — Only fires if both test jobs pass; uploads the static site to GitHub Pages

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

To enable: Go to repo **Settings → Pages → Source → GitHub Actions**.

On PR failures, screenshots from the headless browser test are uploaded as an artifact for debugging.

### Docker

```bash
docker build -t ekf-tuner .
docker run -d -p 8080:80 ekf-tuner
```

Or using Compose:

```bash
docker compose up --build -d
```

The container serves the app on port 80 via Nginx (production-optimized with caching headers).

---

## License

MIT
