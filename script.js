// ============================================================
// EKF SLAM CLASS
// ============================================================
class EKFSlam {
  constructor() {
    this.state = math.matrix([[0], [0], [0], [0]]);
    this.P = math.multiply(math.identity(4), 0.1);
    this.I = math.identity(4);
    this.defaultQ = math.diag([0.5, 0.5, 0.1, 0.05]);
    this.defaultR = math.diag([0.5, 0.5]);
    this.Q = math.clone(this.defaultQ);
    this.R = math.clone(this.defaultR);
  }

  setState(x, y, theta, v) {
    this.state = math.matrix([[x], [y], [theta], [v]]);
  }

  setTuning(qScale, rScale) {
    this.Q = math.multiply(this.defaultQ, qScale);
    this.R = math.multiply(this.defaultR, rScale);
  }

  normalizeAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }

  getX() { return this.state.valueOf()[0][0]; }
  getY() { return this.state.valueOf()[1][0]; }
  getTheta() { return this.state.valueOf()[2][0]; }
  getV() { return this.state.valueOf()[3][0]; }

  f(state, control, dt) {
    const s = state.valueOf();
    const x = s[0][0], y = s[1][0], theta = s[2][0], v = s[3][0];
    const thetaDot = control[0], a = control[1];
    return math.matrix([
      [x + v * Math.cos(theta) * dt],
      [y + v * Math.sin(theta) * dt],
      [this.normalizeAngle(theta + thetaDot * dt)],
      [v + a * dt]
    ]);
  }

  jacobianF(state, control, dt) {
    const s = state.valueOf();
    const theta = s[2][0], v = s[3][0];
    return math.matrix([
      [1, 0, -v * Math.sin(theta) * dt, Math.cos(theta) * dt],
      [0, 1,  v * Math.cos(theta) * dt, Math.sin(theta) * dt],
      [0, 0,  1,                        0                    ],
      [0, 0,  0,                        1                    ]
    ]);
  }

  predict(control, dt) {
    this.state = this.f(this.state, control, dt);
    const thetaIdx = 2;
    this.state.valueOf()[thetaIdx][0] = this.normalizeAngle(this.state.valueOf()[thetaIdx][0]);

    const F = this.jacobianF(this.state, control, dt);
    const FP = math.multiply(F, this.P);
    const FT = math.transpose(F);
    this.P = math.add(math.multiply(FP, FT), this.Q);
    const Pt = math.transpose(this.P);
    this.P = math.multiply(math.add(this.P, Pt), 0.5);
  }

  h(state, landmark) {
    const s = state.valueOf();
    const dx = landmark.x - s[0][0];
    const dy = landmark.y - s[1][0];
    const range = Math.sqrt(dx * dx + dy * dy);
    const bearing = this.normalizeAngle(Math.atan2(dy, dx) - s[2][0]);
    return [range, bearing];
  }

  jacobianH(state, landmark) {
    const s = state.valueOf();
    const dx = landmark.x - s[0][0];
    const dy = landmark.y - s[1][0];
    const range = Math.sqrt(dx * dx + dy * dy);
    if (range < 0.001) return math.zeros([2, 4]);
    return math.matrix([
      [-dx / range, -dy / range,  0, 0],
      [ dy / (range * range), -dx / (range * range), -1, 0]
    ]);
  }

  update(measurement, landmark) {
    const zPred = this.h(this.state, landmark);
    const H = this.jacobianH(this.state, landmark);
    const HP = math.multiply(H, this.P);
    const HT = math.transpose(H);
    const S = math.add(math.multiply(HP, HT), this.R);
    const PHT = math.multiply(this.P, HT);
    const Sinv = math.inv(S);
    const K = math.multiply(PHT, Sinv);

    const z = math.matrix([[measurement[0]], [measurement[1]]]);
    const zp = math.matrix([[zPred[0]], [zPred[1]]]);
    const innov = math.subtract(z, zp);
    let innovArr = innov.valueOf();
    innovArr[1][0] = this.normalizeAngle(innovArr[1][0]);
    const innovFixed = math.matrix(innovArr);

    this.state = math.add(this.state, math.multiply(K, innovFixed));
    this.state.valueOf()[2][0] = this.normalizeAngle(this.state.valueOf()[2][0]);

    const KH = math.multiply(K, H);
    this.P = math.multiply(math.subtract(this.I, KH), this.P);
    const Pt = math.transpose(this.P);
    this.P = math.multiply(math.add(this.P, Pt), 0.5);
  }

  gpsUpdate(measurement) {
    const mx = measurement[0], my = measurement[1];
    const rX = measurement[2] || 1.0, rY = measurement[3] || 1.0;

    const H = math.matrix([[1, 0, 0, 0], [0, 1, 0, 0]]);
    const Rg = math.diag([rX, rY]);
    const z = math.matrix([[mx], [my]]);
    const h = math.matrix([[this.getX()], [this.getY()]]);

    const HT = math.transpose(H);
    const PHT = math.multiply(this.P, HT);
    const S = math.add(math.multiply(H, PHT), Rg);
    const K = math.multiply(PHT, math.inv(S));

    this.state = math.add(this.state, math.multiply(K, math.subtract(z, h)));
    const KH = math.multiply(K, H);
    this.P = math.multiply(math.subtract(this.I, KH), this.P);
    const Pt = math.transpose(this.P);
    this.P = math.multiply(math.add(this.P, Pt), 0.5);
  }

  getPositionUncertainty() {
    const p = this.P.valueOf();
    return p[0][0] + p[1][1];
  }

  getPositionCovariance() {
    const p = this.P.valueOf();
    return [
      [p[0][0], p[0][1]],
      [p[1][0], p[1][1]]
    ];
  }

  getPositionSigma() {
    return Math.sqrt(this.getPositionUncertainty());
  }
}

// ============================================================
// GLOBALS & CONSTANTS
// ============================================================
let ekf;
let running = false;
let completed = false;
let crashed = false;
let simTime = 0;

let trajectory = [];
let landmarks = [];
let racks = [];

let ekfHistory = [];
let trueHistory = [];
const MAX_HISTORY = 200;

let lastMeasurement = null;
let lastLandmark = null;
let lastMeasPos = null;

let currentUncertainty = 0;
let currentDivergence = 0;
let maxDivergence = 0;
let currentSavings = 0;

const DT = 1;
const TOTAL_TIME = 400;
const CORRIDOR_T_START = 120;
const CORRIDOR_T_END = 280;
const ROBOT_RADIUS = 16;
const MAX_SENSOR_RANGE = 400;
const LIDAR_COST = 5000;
const MAX_SAVINGS_SIGMA = 3.0;

let sensorWheelEnabled = true;
let sensorWheelAccuracy = 1.0;
let sensorLidarEnabled = true;
let sensorLidarAccuracy = 1.0;
let sensorGpsEnabled = false;
let sensorGpsAccuracy = 1.0;

let externalSensors = [];
let externalSensorIdCounter = 0;
let placingSensor = false;
let dragSensorId = null;

function updateSensorTuning() {
  sensorWheelAccuracy = parseFloat(document.getElementById('wheelOdometryAccuracy').value) || 1.0;
  sensorLidarAccuracy = parseFloat(document.getElementById('lidarAccuracy').value) || 1.0;
  sensorGpsAccuracy = parseFloat(document.getElementById('gpsAccuracy').value) || 1.0;
  sensorWheelEnabled = document.getElementById('sensorWheelOdometry').checked;
  sensorLidarEnabled = document.getElementById('sensorLidar').checked;
  sensorGpsEnabled = document.getElementById('sensorGps').checked;

  const wheelScale = sensorWheelEnabled ? (1.0 / Math.max(sensorWheelAccuracy, 0.01)) : 50;
  const lidarScale = sensorLidarEnabled ? (1.0 / Math.max(sensorLidarAccuracy, 0.01)) : 100;

  ekf.Q = math.multiply(ekf.defaultQ, wheelScale);
  ekf.R = math.multiply(ekf.defaultR, lidarScale);
}

let slipMode = 'deterministic';
let slipOffset = 0;
let slipDuration = CORRIDOR_T_END - CORRIDOR_T_START;
let slipStartTime = CORRIDOR_T_START;
let slipEndTime = CORRIDOR_T_END;

let agvSpeed = 2.0;
let startPoint = { x: 150, y: 440 };
let endPoint = { x: 700, y: 160 };
let userCorridorStart = CORRIDOR_T_START;
let userCorridorEnd = CORRIDOR_T_END;

// ============================================================
// TRAJECTORY & WORLD
// ============================================================
function cubicBezier(p0, p1, p2, p3, t) {
  const u = 1 - t;
  return {
    x: u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x,
    y: u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y
  };
}

function buildTrajectory() {
  trajectory = [];
  const p0 = { x: startPoint.x, y: startPoint.y };
  const dx = endPoint.x - startPoint.x;
  const dy = endPoint.y - startPoint.y;
  // Control-point offsets preserved proportionally from original layout
  const p1 = { x: startPoint.x + dx * (130 / 550), y: startPoint.y };
  const p2 = { x: endPoint.x - dx * (200 / 550), y: endPoint.y + Math.abs(dy) * (140 / 280) };
  const p3 = { x: endPoint.x, y: endPoint.y };

  for (let i = 0; i <= TOTAL_TIME; i++) {
    const t = i / TOTAL_TIME;
    const pt = cubicBezier(p0, p1, p2, p3, t);
    trajectory.push({ x: pt.x, y: pt.y, theta: 0, v: agvSpeed });
  }

  for (let i = 0; i < trajectory.length; i++) {
    if (i < trajectory.length - 1) {
      const dx = trajectory[i + 1].x - trajectory[i].x;
      const dy = trajectory[i + 1].y - trajectory[i].y;
      trajectory[i].theta = Math.atan2(dy, dx);
    } else {
      trajectory[i].theta = trajectory[i - 1].theta;
    }
  }
}

function buildLandmarks() {
  if (externalSensors.length > 0 || externalSensorIdCounter > 0) return;
  const defaultPositions = [
    { x: 100, y: 480 }, { x: 100, y: 300 },
    { x: 300, y: 100 }, { x: 600, y: 480 }, { x: 750, y: 100 }
  ];
  for (const pos of defaultPositions) {
    externalSensors.push({ id: externalSensorIdCounter++, x: pos.x, y: pos.y, accuracy: 1.0 });
  }
  renderExternalSensorList();
}

function buildRacks() {
  racks = [];

  // Racks offset ~40–50px to the RIGHT (outside) of the trajectory curve.
  // A well-tracking EKF stays within 10px of the trajectory and passes safely.
  // A diverged EKF that under-steers (drifts right) hits these racks.
  const rackPositions = [
    // Right-side wall — catches the diverged EKF during & after the corridor
    { x: 420, y: 370, w: 30, h: 18 },
    { x: 450, y: 355, w: 30, h: 18 },
    { x: 480, y: 340, w: 30, h: 18 },
    { x: 510, y: 325, w: 30, h: 18 },
    { x: 540, y: 310, w: 30, h: 18 },
    { x: 570, y: 295, w: 30, h: 18 },
    { x: 600, y: 280, w: 30, h: 18 },
    // Wider corridor-section racks (larger collision zone)
    { x: 440, y: 385, w: 18, h: 30 },
    { x: 490, y: 360, w: 18, h: 30 },
    { x: 540, y: 335, w: 18, h: 30 },
    // Visual context racks near start (safe, parallel to path)
    { x: 180, y: 370, w: 28, h: 14 },
    { x: 180, y: 330, w: 28, h: 14 },
    { x: 180, y: 290, w: 28, h: 14 },
  ];

  for (const r of rackPositions) {
    racks.push(r);
  }
}

function computeSlipParams() {
  if (slipMode === 'deterministic') {
    slipStartTime = userCorridorStart;
    slipEndTime = userCorridorEnd;
    slipDuration = slipEndTime - slipStartTime;
  } else {
    slipOffset = Math.floor(Math.random() * 60);
    const endOffset = Math.floor(Math.random() * 40);
    slipStartTime = CORRIDOR_T_START + slipOffset;
    slipEndTime = CORRIDOR_T_END - endOffset;
    slipDuration = slipEndTime - slipStartTime;
  }
}

function getNominalState(t) {
  const idx = Math.min(Math.floor(t), TOTAL_TIME);
  const frac = t - Math.floor(t);
  if (idx >= TOTAL_TIME) return { ...trajectory[TOTAL_TIME] };
  const next = Math.min(idx + 1, TOTAL_TIME);
  return {
    x: trajectory[idx].x + (trajectory[next].x - trajectory[idx].x) * frac,
    y: trajectory[idx].y + (trajectory[next].y - trajectory[idx].y) * frac,
    theta: trajectory[idx].theta,
    v: trajectory[idx].v
  };
}

function isInCorridor(t) {
  return t >= slipStartTime && t <= slipEndTime;
}

function getCorruptedControl(t) {
  const idx = Math.min(Math.floor(t), TOTAL_TIME - 1);
  const nextIdx = Math.min(idx + 1, TOTAL_TIME - 1);

  let dTheta = trajectory[nextIdx].theta - trajectory[idx].theta;
  dTheta = Math.atan2(Math.sin(dTheta), Math.cos(dTheta));
  let thetaDot = dTheta / DT;
  const a = 0;

  if (isInCorridor(t)) {
    // Slip: corrupt steering — robot thinks it's not turning enough
    // This causes the EKF to predict a straighter path than reality
    const corridorProgress = (t - slipStartTime) / slipDuration;
    const slipMag = 0.015 * Math.sin(corridorProgress * Math.PI);
    thetaDot -= slipMag;
    thetaDot += (Math.random() - 0.5) * 0.002;
  }

  return [thetaDot, a];
}

function getTrueState(t) {
  const nominal = getNominalState(t);
  return { ...nominal };
}

function getVisibleLandmarks(state) {
  if (isInCorridor(simTime)) return [];
  return externalSensors.filter(s => {
    const dx = s.x - state.x;
    const dy = s.y - state.y;
    return Math.sqrt(dx * dx + dy * dy) < MAX_SENSOR_RANGE;
  });
}

function getNearestLandmark(state, visible) {
  let nearest = null;
  let minDist = Infinity;
  for (const lm of visible) {
    const dx = lm.x - state.x;
    const dy = lm.y - state.y;
    const d = Math.sqrt(dx * dx + dy * dy);
    if (d < minDist) {
      minDist = d;
      nearest = lm;
    }
  }
  return nearest;
}

function noisyMeasurement(trueState, landmark, ekfInstance) {
  const filter = ekfInstance || ekf;
  if (!filter) return [0, 0];
  const rVal = filter.R.valueOf();
  const rangeNoise = rVal[0][0] || 0.5;
  const bearingNoise = rVal[1][1] || 0.5;
  const dx = landmark.x - trueState.x;
  const dy = landmark.y - trueState.y;
  const trueRange = Math.sqrt(dx * dx + dy * dy);
  const trueBearing = Math.atan2(dy, dx) - trueState.theta;
  return [
    trueRange + (Math.random() - 0.5) * 2 * rangeNoise,
    trueBearing + (Math.random() - 0.5) * 2 * bearingNoise * 0.05
  ];
}

function measurementToPosition(measurement, landmark) {
  if (!measurement || !landmark) return null;
  const range = measurement[0];
  const bearing = measurement[1];
  const ex = landmark.x - range * Math.cos(bearing);
  const ey = landmark.y - range * Math.sin(bearing);
  return { x: ex, y: ey };
}

// ============================================================
// p5.js SETUP
// ============================================================
function setup() {
  const canvas = createCanvas(780, 520);
  canvas.parent('p5canvas');
  pixelDensity(1);
  colorMode(RGB, 255);
  angleMode(RADIANS);
  rectMode(CENTER);

  buildTrajectory();
  buildLandmarks();
  buildRacks();
  initSimulation();
}

function initSimulation() {
  ekf = new EKFSlam();
  const start = trajectory[0];
  ekf.setState(start.x, start.y, start.theta, start.v);
  ekfHistory = [];
  trueHistory = [];
  simTime = 0;
  completed = false;
  crashed = false;
  running = false;
  lastMeasurement = null;
  lastLandmark = null;
  lastMeasPos = null;
  currentUncertainty = 0;
  currentDivergence = 0;
  maxDivergence = 0;
  currentSavings = 0;
  computeSlipParams();
  updateSensorTuning();
  updateSliderDisplay();
  updateMetrics();

  document.getElementById('resultPanel').style.display = 'none';
  document.getElementById('flashRed').classList.remove('show');
}

let dragTarget = null;

function getNearestTrajectoryIndex(mx, my) {
  let minDist = Infinity;
  let nearest = 0;
  for (let i = 0; i < trajectory.length; i++) {
    const d = Math.hypot(mx - trajectory[i].x, my - trajectory[i].y);
    if (d < minDist) {
      minDist = d;
      nearest = i;
    }
  }
  return nearest;
}

function mousePressed() {
  if (running || completed || crashed) return;

  // Placing a new external sensor
  if (placingSensor) {
    externalSensors.push({
      id: externalSensorIdCounter++,
      x: constrain(mouseX, 20, width - 20),
      y: constrain(mouseY, 20, height - 20),
      accuracy: 1.0
    });
    placingSensor = false;
    renderExternalSensorList();
    document.getElementById('btnAddBeacon').textContent = '+ Add Beacon';
    return false;
  }

  // Drag external sensors
  for (const s of externalSensors) {
    if (Math.hypot(mouseX - s.x, mouseY - s.y) < 18) {
      dragTarget = 'externalSensor';
      dragSensorId = s.id;
      return false;
    }
  }

  const dStart = Math.hypot(mouseX - startPoint.x, mouseY - startPoint.y);
  const dEnd = Math.hypot(mouseX - endPoint.x, mouseY - endPoint.y);
  if (dStart < 24) {
    dragTarget = 'start';
    return false;
  }
  if (dEnd < 24) {
    dragTarget = 'end';
    return false;
  }

  // Clicking on the AGV toggles LIDAR
  if (dStart >= 24 && dEnd >= 24) {
    const agvDist = Math.hypot(mouseX - ekf.getX(), mouseY - ekf.getY());
    if (agvDist < ROBOT_RADIUS * 2) {
      const toggle = document.getElementById('sensorLidar');
      if (toggle) {
        toggle.checked = !toggle.checked;
        updateSensorTuning();
        updateMetrics();
      }
      return false;
    }
  }
  if (slipMode === 'deterministic' && trajectory.length) {
    const sIdx = Math.min(Math.floor(slipStartTime), trajectory.length - 1);
    const eIdx = Math.min(Math.floor(slipEndTime), trajectory.length - 1);
    const sp = trajectory[sIdx];
    const ep = trajectory[eIdx];
    const dCS = Math.hypot(mouseX - sp.x, mouseY - sp.y);
    const dCE = Math.hypot(mouseX - ep.x, mouseY - ep.y);
    if (dCS < 20) {
      dragTarget = 'corridorStart';
      return false;
    }
    if (dCE < 20) {
      dragTarget = 'corridorEnd';
      return false;
    }
  }
}

function mouseDragged() {
  if (!dragTarget) return;
  const x = constrain(mouseX, 20, width - 20);
  const y = constrain(mouseY, 20, height - 20);
  if (dragTarget === 'start') {
    startPoint.x = x;
    startPoint.y = y;
    buildTrajectory();
    initSimulation();
  } else if (dragTarget === 'end') {
    endPoint.x = x;
    endPoint.y = y;
    buildTrajectory();
    initSimulation();
  } else if (dragTarget === 'corridorStart' || dragTarget === 'corridorEnd') {
    const idx = getNearestTrajectoryIndex(mouseX, mouseY);
    if (dragTarget === 'corridorStart') {
      userCorridorStart = Math.min(idx, userCorridorEnd - 10);
    } else {
      userCorridorEnd = Math.max(idx, userCorridorStart + 10);
    }
    computeSlipParams();
  } else if (dragTarget === 'externalSensor') {
    const s = externalSensors.find(s => s.id === dragSensorId);
    if (s) {
      s.x = constrain(mouseX, 20, width - 20);
      s.y = constrain(mouseY, 20, height - 20);
      renderExternalSensorList();
    }
  }
}

function mouseReleased() {
  dragTarget = null;
  dragSensorId = null;
}

// ============================================================
// p5.js DRAW
// ============================================================
function draw() {
  background(30, 35, 42);

  // Always draw the warehouse
  drawWarehouse();

  if (completed || crashed) {
    drawRacks();
    drawLandmarks();
    drawExternalSensors();
    drawStartEnd();
    drawTrajectoryPath();
    drawTruePath();
    drawEKFPath();
    if (simTime >= slipStartTime - 20 && simTime <= slipEndTime + 20) {
      drawCorridorZone();
    }
    renderAllStates();
    return;
  }

  if (!running) {
    drawRacks();
    drawLandmarks();
    drawExternalSensors();
    drawStartEnd();
    drawTrajectoryPath();
    drawCorridorMarkers();
    drawEKFEstimate();
    drawIdlePrompt();
    return;
  }

  if (simTime >= TOTAL_TIME) {
    routeComplete();
    return;
  }

  // --- SIMULATION STEP ---
  const trueSt = getTrueState(simTime);
  const control = getCorruptedControl(simTime);

  // EKF predict
  ekf.predict(control, DT);

  // Get visible landmarks & update (LIDAR)
  if (sensorLidarEnabled) {
    const visible = getVisibleLandmarks(trueSt);
    const nearestLm = getNearestLandmark(trueSt, visible);
    if (nearestLm) {
      const meas = noisyMeasurement(trueSt, nearestLm);
      ekf.update(meas, nearestLm);
      lastMeasurement = meas;
      lastLandmark = nearestLm;
      lastMeasPos = measurementToPosition(meas, nearestLm);
    } else {
      lastMeasurement = null;
      lastLandmark = null;
      lastMeasPos = null;
    }
  } else {
    lastMeasurement = null;
    lastLandmark = null;
    lastMeasPos = null;
  }

  // GPS update
  if (sensorGpsEnabled && !isInCorridor(simTime)) {
    const gpsNoise = 1.0 / Math.max(sensorGpsAccuracy, 0.01);
    const gpsMeas = [
      trueSt.x + (Math.random() - 0.5) * gpsNoise * 4,
      trueSt.y + (Math.random() - 0.5) * gpsNoise * 4,
      gpsNoise * 2,
      gpsNoise * 2
    ];
    ekf.gpsUpdate(gpsMeas);
  }

  // External sensor updates (position beacons)
  if (sensorLidarEnabled && !isInCorridor(simTime)) {
  for (const s of externalSensors) {
    const dx = trueSt.x - s.x;
    const dy = trueSt.y - s.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist < MAX_SENSOR_RANGE) {
      const noise = 1.0 / Math.max(s.accuracy, 0.01);
      const meas = [
        trueSt.x + (Math.random() - 0.5) * noise * 4,
        trueSt.y + (Math.random() - 0.5) * noise * 4,
        noise * 2,
        noise * 2
      ];
      ekf.gpsUpdate(meas);
    }
  }
  }

  // Compute divergence
  const ex = ekf.getX() - trueSt.x;
  const ey = ekf.getY() - trueSt.y;
  currentDivergence = Math.sqrt(ex * ex + ey * ey);
  if (currentDivergence > maxDivergence) maxDivergence = currentDivergence;
  currentUncertainty = ekf.getPositionUncertainty();
  currentSavings = calculateSavings();

  // Divergence check (primary failure mode for high-R / low-Q tunings)
  if (checkDivergence()) {
    gameOver('divergence');
    return;
  }

  // Collision check (secondary visual signal)
  if (checkCollision()) {
    gameOver('collision');
    return;
  }

  // Lost estimate check (EKF diverged off-canvas)
  if (ekf.getX() < -50 || ekf.getX() > width + 50 ||
      ekf.getY() < -50 || ekf.getY() > height + 50) {
    gameOver('lost');
    return;
  }

  // History
  ekfHistory.push({ x: ekf.getX(), y: ekf.getY(), theta: ekf.getTheta() });
  trueHistory.push({ x: trueSt.x, y: trueSt.y, theta: trueSt.theta });
  if (ekfHistory.length > MAX_HISTORY) ekfHistory.shift();
  if (trueHistory.length > MAX_HISTORY) trueHistory.shift();

  simTime += DT * (agvSpeed / 2.0);

  // --- RENDER ---
  drawRacks();
  drawLandmarks();
  drawExternalSensors();
  drawStartEnd();
  drawTrajectoryPath();
  drawTruePath();
  drawEKFPath();
  if (lastMeasurement && lastLandmark) {
    drawMeasurement(lastMeasurement, lastLandmark);
  }
  drawTrueState(trueSt);
  drawEKFEstimate();
  drawCorridorZone();

  updateMetrics();
}

// ============================================================
// RENDERING
// ============================================================
function drawWarehouse() {
  // Floor
  fill(36, 42, 50);
  noStroke();
  rect(width / 2, height / 2, width, height);

  // Grid
  stroke(45, 52, 62);
  strokeWeight(0.5);
  for (let x = 0; x <= width; x += 40) {
    line(x, 0, x, height);
  }
  for (let y = 0; y <= height; y += 40) {
    line(0, y, width, y);
  }
}

function drawRacks() {
  for (const rack of racks) {
    // Shelf body
    fill(60, 70, 85);
    stroke(80, 92, 110);
    strokeWeight(1);
    rect(rack.x, rack.y, rack.w, rack.h, 3);

    // Hazard stripes
    for (let i = -rack.w / 2; i < rack.w / 2; i += 8) {
      fill(231, 76, 60, 60);
      noStroke();
      rect(rack.x + i, rack.y, 4, rack.h, 1);
    }
  }
}

function drawLandmarks() {}

function drawStartEnd() {
  const sx = startPoint.x, sy = startPoint.y;
  const ex = endPoint.x, ey = endPoint.y;
  const idle = !running && !completed && !crashed;

  const hoverS = idle && Math.hypot(mouseX - sx, mouseY - sy) < 24;
  const hoverE = idle && Math.hypot(mouseX - ex, mouseY - ey) < 24;

  // Start zone (Point A)
  noStroke();
  fill(46, 204, 113, hoverS ? 60 : 30);
  circle(sx, sy, hoverS ? 80 : 60);
  fill(46, 204, 113, hoverS ? 90 : 60);
  circle(sx, sy, hoverS ? 40 : 30);
  fill(46, 204, 113, hoverS ? 220 : 180);
  circle(sx, sy, hoverS ? 12 : 8);
  if (hoverS) {
    noFill();
    stroke(46, 204, 113, 80);
    strokeWeight(2);
    circle(sx, sy, 28);
    noStroke();
  }
  fill(46, 204, 113, 180);
  textAlign(CENTER, CENTER);
  textSize(10);
  text('A', sx, sy - 14);

  // End zone (Point B)
  fill(52, 152, 219, hoverE ? 60 : 30);
  circle(ex, ey, hoverE ? 80 : 60);
  fill(52, 152, 219, hoverE ? 90 : 60);
  circle(ex, ey, hoverE ? 40 : 30);
  fill(52, 152, 219, hoverE ? 220 : 180);
  circle(ex, ey, hoverE ? 12 : 8);
  if (hoverE) {
    noFill();
    stroke(52, 152, 219, 80);
    strokeWeight(2);
    circle(ex, ey, 28);
    noStroke();
  }
  fill(52, 152, 219, 180);
  text('B', ex, ey - 14);
}

function drawCorridorMarkers() {
  if (slipMode !== 'deterministic' || running || completed || crashed) return;
  if (!trajectory.length) return;
  const sIdx = Math.min(Math.floor(slipStartTime), trajectory.length - 1);
  const eIdx = Math.min(Math.floor(slipEndTime), trajectory.length - 1);

  const hoverS = Math.hypot(mouseX - trajectory[sIdx].x, mouseY - trajectory[sIdx].y) < 20;
  const hoverE = Math.hypot(mouseX - trajectory[eIdx].x, mouseY - trajectory[eIdx].y) < 20;

  // Highlighted corridor segment
  noFill();
  stroke(243, 156, 18, 80);
  strokeWeight(6);
  beginShape();
  for (let i = sIdx; i <= eIdx; i++) {
    vertex(trajectory[i].x, trajectory[i].y);
  }
  endShape();

  // Start marker
  const sx = trajectory[sIdx].x, sy = trajectory[sIdx].y;
  noStroke();
  fill(243, 156, 18, hoverS ? 200 : 150);
  const sz = hoverS ? 14 : 10;
  quad(sx - sz, sy, sx, sy - sz * 1.2, sx + sz, sy, sx, sy + sz * 1.2);
  if (hoverS) {
    noFill();
    stroke(243, 156, 18, 100);
    strokeWeight(2);
    circle(sx, sy, 28);
    noStroke();
  }
  fill(243, 156, 18, 200);
  textAlign(CENTER, CENTER);
  textSize(9);
  text('▸', sx, sy - 16);

  // End marker
  const ex = trajectory[eIdx].x, ey = trajectory[eIdx].y;
  fill(231, 76, 60, hoverE ? 200 : 150);
  const ez = hoverE ? 14 : 10;
  quad(ex - ez, ey, ex, ey - ez * 1.2, ex + ez, ey, ex, ey + ez * 1.2);
  if (hoverE) {
    noFill();
    stroke(231, 76, 60, 100);
    strokeWeight(2);
    circle(ex, ey, 28);
    noStroke();
  }
  fill(231, 76, 60, 200);
  text('◂', ex, ey - 16);

  // Label
  fill(243, 156, 18, 140);
  textAlign(CENTER, TOP);
  textSize(9);
  const mid = trajectory[Math.floor((sIdx + eIdx) / 2)];
  text('featureless corridor', mid.x, mid.y + 16);
}

function drawExternalSensors() {
  for (const s of externalSensors) {
    const hover = !running && !completed && !crashed && Math.hypot(mouseX - s.x, mouseY - s.y) < 18;

    // Sensor icon
    noStroke();
    fill(155, 89, 182, hover ? 220 : 160);
    circle(s.x, s.y, hover ? 22 : 16);
    fill(255, 255, 255, hover ? 220 : 160);
    textAlign(CENTER, CENTER);
    textSize(hover ? 11 : 9);
    text('\u2699', s.x, s.y + 1);

    // Label
    if (hover) {
      fill(155, 89, 182, 180);
      noStroke();
      textSize(8);
      textAlign(CENTER, TOP);
      text('ext. sensor', s.x, s.y + 16);
    }
  }
}

function renderExternalSensorList() {
  const container = document.getElementById('externalSensorsList');
  if (!container) return;
  container.innerHTML = '';
  for (const s of externalSensors) {
    const row = document.createElement('div');
    row.className = 'sensor-row';
    row.innerHTML =
      '<span class="sensor-id">Beacon ' + (s.id + 1) + '</span>' +
      '<input type="range" class="sensor-slider" min="0.1" max="5.0" step="0.01" value="' + s.accuracy.toFixed(2) + '">' +
      '<span class="sensor-value" id="extVal' + s.id + '">' + s.accuracy.toFixed(2) + '</span>' +
      '<button class="btn-remove-sensor" data-id="' + s.id + '">&times;</button>';
    const slider = row.querySelector('.sensor-slider');
    const valSpan = row.querySelector('.sensor-value');
    slider.addEventListener('input', function () {
      s.accuracy = parseFloat(this.value);
      valSpan.textContent = s.accuracy.toFixed(2);
    });
    row.querySelector('.btn-remove-sensor').addEventListener('click', function () {
      externalSensors = externalSensors.filter(ext => ext.id !== s.id);
      renderExternalSensorList();
    });
    container.appendChild(row);
  }
}

function drawTrajectoryPath() {
  noFill();
  stroke(255, 255, 255, 30);
  strokeWeight(2);
  beginShape();
  for (const pt of trajectory) {
    vertex(pt.x, pt.y);
  }
  endShape();
}

function drawCorridorZone() {
  if (simTime < slipStartTime - 20 || simTime > slipEndTime + 20) return;

  const idx1 = Math.max(0, Math.floor(slipStartTime));
  const idx2 = Math.min(trajectory.length - 1, Math.ceil(slipEndTime));
  const pts = trajectory.slice(idx1, idx2 + 1);

  if (pts.length < 2) return;

  // Draw a hazard glow along the corridor
  noFill();
  for (const pt of pts) {
    fill(243, 156, 18, 20 - 15 * Math.abs(simTime - (slipStartTime + slipEndTime) / 2) / (slipEndTime - slipStartTime));
    noStroke();
    circle(pt.x, pt.y, 40);
  }

  // Warning label
  if (simTime >= slipStartTime && simTime <= slipEndTime + 10) {
    fill(243, 156, 18, 180);
    noStroke();
    textAlign(CENTER, BOTTOM);
    textSize(11);
    textFont('monospace');
    text('⚠ FEATURELESS CORRIDOR — NO LANDMARKS', width / 2, height - 20);
    textFont('sans-serif');
  }
}

function drawTrueState(state) {
  // Ghost AGV
  push();
  translate(state.x, state.y);
  rotate(state.theta);

  // Body
  noFill();
  stroke(200, 200, 200, 60);
  strokeWeight(1.5);
  rect(0, 0, 32, 18, 4);

  // Direction indicator
  stroke(200, 200, 200, 40);
  strokeWeight(1);
  line(0, 0, 20, 0);

  // Label
  fill(200, 200, 200, 30);
  noStroke();
  textAlign(CENTER, CENTER);
  textSize(8);
  text('TRUE', 0, -16);
  pop();
}

function drawMeasurement(measurement, landmark) {
  if (!lastMeasPos) return;

  // Noisy measurement dots
  noStroke();
  fill(231, 76, 60, 80);
  circle(lastMeasPos.x, lastMeasPos.y, 10);

  fill(231, 76, 60, 150);
  circle(lastMeasPos.x, lastMeasPos.y, 4);

  // Line from measurement to landmark
  stroke(231, 76, 60, 30);
  strokeWeight(1);
  line(lastMeasPos.x, lastMeasPos.y, landmark.x, landmark.y);

  // Label
  fill(231, 76, 60, 120);
  noStroke();
  textAlign(LEFT, BOTTOM);
  textSize(8);
  text('RAW SENSOR', lastMeasPos.x + 8, lastMeasPos.y - 4);
}

function drawEKFEstimate() {
  const x = ekf.getX();
  const y = ekf.getY();
  const theta = ekf.getTheta();

  push();
  translate(x, y);
  rotate(theta);

  // Glow
  noStroke();
  fill(52, 152, 219, 25);
  rect(0, 0, 38, 24, 6);

  // Body
  fill(44, 62, 80, 220);
  stroke(52, 152, 219, 200);
  strokeWeight(2);
  rect(0, 0, 32, 18, 4);

  // Windshield
  fill(52, 152, 219, 60);
  noStroke();
  rect(8, 0, 8, 12, 2);

  // Direction indicator
  stroke(52, 152, 219, 180);
  strokeWeight(2);
  line(0, 0, 22, 0);
  line(22, 0, 18, -4);
  line(22, 0, 18, 4);

  // Label
  fill(52, 152, 219, 150);
  noStroke();
  textAlign(CENTER, CENTER);
  textSize(8);
  text('EKF', 0, -16);

  pop();

  // LIDAR range visualization
  if (sensorLidarEnabled) {
    noFill();
    stroke(52, 152, 219, 30);
    strokeWeight(1);
    circle(x, y, MAX_SENSOR_RANGE * 2);

    // Sweep arc
    const sweepAngle = (frameCount * 0.02) % (Math.PI * 2);
    noFill();
    stroke(52, 152, 219, 50);
    strokeWeight(2);
    arc(x, y, MAX_SENSOR_RANGE * 2, MAX_SENSOR_RANGE * 2,
        sweepAngle - 0.3, sweepAngle + 0.3);

    stroke(52, 152, 219, 80);
    strokeWeight(1);
    line(x, y,
         x + Math.cos(sweepAngle) * MAX_SENSOR_RANGE,
         y + Math.sin(sweepAngle) * MAX_SENSOR_RANGE);
  }

  // Sensor status indicators (drawn in world coords)
  noStroke();
  textAlign(LEFT, CENTER);
  textSize(8);

  let statusX = x + 22;
  let statusY = y - 14;
  if (sensorLidarEnabled) {
    fill(52, 152, 219, 180);
    text('\u25B3', statusX, statusY);
    statusX += 12;
  }
  if (sensorGpsEnabled) {
    fill(46, 204, 113, 180);
    text('\u25C9', statusX, statusY);
    statusX += 12;
  }
  if (sensorWheelEnabled) {
    fill(155, 89, 182, 180);
    text('\u25A0', statusX, statusY);
  }
}

function drawTruePath() {
  if (trueHistory.length < 2) return;
  noFill();
  stroke(200, 200, 200, 60);
  strokeWeight(1.5);
  beginShape();
  for (const pt of trueHistory) {
    vertex(pt.x, pt.y);
  }
  endShape();
}

function drawEKFPath() {
  if (ekfHistory.length < 2) return;
  noFill();
  stroke(52, 152, 219, 120);
  strokeWeight(2);
  beginShape();
  for (const pt of ekfHistory) {
    vertex(pt.x, pt.y);
  }
  endShape();
}

function renderAllStates() {
  // Render all EKF history
  for (let i = 0; i < ekfHistory.length; i++) {
    const pt = ekfHistory[i];
    const alpha = map(i, 0, ekfHistory.length, 30, 180);
    push();
    translate(pt.x, pt.y);
    rotate(pt.theta);
    noStroke();
    fill(52, 152, 219, alpha * 0.3);
    rect(0, 0, 30, 16, 3);
    pop();
  }

  // Final EKF state
  if (ekfHistory.length > 0) {
    const last = ekfHistory[ekfHistory.length - 1];
    drawEKFEstimatePos(last.x, last.y, last.theta);
  }
}

function drawEKFEstimatePos(x, y, theta) {
  push();
  translate(x, y);
  rotate(theta);
  fill(44, 62, 80, 220);
  stroke(52, 152, 219, 200);
  strokeWeight(2);
  rect(0, 0, 32, 18, 4);
  stroke(52, 152, 219, 180);
  strokeWeight(2);
  line(0, 0, 22, 0);
  line(22, 0, 18, -4);
  line(22, 0, 18, 4);
  pop();
}

function drawIdlePrompt() {
  fill(200, 200, 200, 40);
  noStroke();
  textAlign(CENTER, CENTER);
  textSize(16);
  textFont('sans-serif');
  text('Tune the sliders →\nthen press DEPLOY AGV', width / 2, height / 2);
  textFont('sans-serif');
}

// ============================================================
// COLLISION & GAME STATE
// ============================================================
function checkCollision() {
  // Grace period after corridor exit to allow EKF to reconverge
  if (simTime > slipEndTime && simTime < slipEndTime + 8) return false;

  const x = ekf.getX();
  const y = ekf.getY();
  const margin = ROBOT_RADIUS;
  for (const rack of racks) {
    if (x + margin > rack.x - rack.w / 2 &&
        x - margin < rack.x + rack.w / 2 &&
        y + margin > rack.y - rack.h / 2 &&
        y - margin < rack.y + rack.h / 2) {
      return true;
    }
  }
  return false;
}

function checkDivergence() {
  const allow = document.getElementById('allowDivergence');
  if (allow && !allow.checked) return false;
  return currentDivergence > 60;
}

function gameOver(reason) {
  crashed = true;
  running = false;

  const container = document.getElementById('canvasContainer');
  container.classList.add('shake');
  setTimeout(() => container.classList.remove('shake'), 500);

  const flash = document.getElementById('flashRed');
  flash.classList.add('show');
  setTimeout(() => flash.classList.remove('show'), 800);

  const panel = document.getElementById('resultPanel');
  const content = document.getElementById('resultContent');
  if (reason === 'collision') {
    content.innerHTML =
      '<h2 style="color: var(--danger); margin-bottom: 8px;">✗ COLLISION DETECTED</h2>' +
      '<p style="color: var(--text-secondary); margin-bottom: 16px;">The EKF estimate drifted into warehouse racking. Try increasing Q (trust odometry less) or reducing R (trust sensors more).</p>' +
      '<button class="btn-restart" onclick="resetSimulation()" style="padding: 10px 28px; border: 2px solid var(--accent); border-radius: 8px; background: transparent; color: var(--accent); font-weight: 600; cursor: pointer;">TUNE AGAIN</button>';
  } else {
    content.innerHTML =
      '<h2 style="color: var(--danger); margin-bottom: 8px;">✗ ESTIMATE DIVERGED</h2>' +
      '<p style="color: var(--text-secondary); margin-bottom: 16px;">The estimated state diverged from the true position. Review your Q/R tuning.</p>' +
      '<button class="btn-restart" onclick="resetSimulation()" style="padding: 10px 28px; border: 2px solid var(--accent); border-radius: 8px; background: transparent; color: var(--accent); font-weight: 600; cursor: pointer;">TUNE AGAIN</button>';
  }
  panel.style.display = 'block';
  updateMetrics();
}

function routeComplete() {
  completed = true;
  running = false;
  const panel = document.getElementById('resultPanel');
  const content = document.getElementById('resultContent');
  content.innerHTML =
    '<h2 style="color: var(--success); margin-bottom: 8px;">✓ ROUTE COMPLETE</h2>' +
    '<p style="color: var(--text-secondary); margin-bottom: 16px;">' +
    `Max divergence: ${(maxDivergence).toFixed(1)}px  ·  ` +
    `Sensor savings: ${(currentSavings * 100).toFixed(0)}%  ·  ` +
    `Final uncertainty: ${currentUncertainty.toFixed(3)}` +
    '</p>' +
    '<button class="btn-restart" onclick="resetSimulation()" style="padding: 10px 28px; border: 2px solid var(--accent); border-radius: 8px; background: transparent; color: var(--accent); font-weight: 600; cursor: pointer;">TUNE AGAIN</button>';
  panel.style.display = 'block';
  updateMetrics();
}

// ============================================================
// SAVINGS CALCULATOR
// ============================================================
function calculateSavings(ekfInstance) {
  const filter = ekfInstance || ekf;
  if (!filter) return 0;
  const sigma = Math.min(filter.getPositionSigma(), MAX_SAVINGS_SIGMA);
  return 0.40 * Math.max(0, 1 - sigma / MAX_SAVINGS_SIGMA);
}

// ============================================================
// DASHBOARD METRICS
// ============================================================
function updateMetrics() {
  const uncertaintyEl = document.getElementById('uncertaintyDisplay');
  const divergenceEl = document.getElementById('divergenceDisplay');
  const savingsPercentEl = document.getElementById('savingsPercent');
  const savingsBarEl = document.getElementById('savingsBar');
  const savingsNarrativeEl = document.getElementById('savingsNarrative');
  const statusTextEl = document.getElementById('statusText');
  const statusLedEl = document.getElementById('statusLed');

  if (!running && !completed && !crashed) {
    uncertaintyEl.textContent = '—';
    divergenceEl.textContent = '—';
    savingsPercentEl.textContent = '0%';
    savingsBarEl.style.width = '0%';
    statusTextEl.textContent = 'Awaiting Deployment';
    statusLedEl.className = 'status-led green';
    return;
  }

  if (crashed) {
    statusTextEl.textContent = 'CRASHED — Tune & Retry';
    statusLedEl.className = 'status-led red';
    return;
  }

  if (completed) {
    statusTextEl.textContent = '✓ Route Complete';
    statusLedEl.className = 'status-led green';
    return;
  }

  const u = ekf.getPositionUncertainty();
  const d = currentDivergence;
  const savings = currentSavings;

  uncertaintyEl.textContent = u.toFixed(3);
  uncertaintyEl.className = 'metric-value ' + (u < 1.0 ? 'good' : u < 3.0 ? 'warn' : 'bad');

  divergenceEl.textContent = d.toFixed(1) + ' px';
  divergenceEl.className = 'metric-value ' + (d < 30 ? 'good' : d < 60 ? 'warn' : 'bad');

  const savingsPct = (savings * 100).toFixed(0);
  savingsPercentEl.textContent = savingsPct + '%';
  savingsBarEl.style.width = savingsPct + '%';

  const dollarSavings = Math.round(savings * LIDAR_COST);
  savingsNarrativeEl.innerHTML =
    `At this tuning level, each AGV saves approximately <strong>~$${dollarSavings}</strong> ` +
    `in lidar specification costs, reducing BOM by <strong>${savingsPct}%</strong> per unit.`;

  // Status
  if (isInCorridor(simTime)) {
    statusTextEl.textContent = '⚠ Corridor — No Landmarks';
    statusLedEl.className = 'status-led yellow';
  } else if (d > 50) {
    statusTextEl.textContent = '⚠ Diverging — Risk of Collision';
    statusLedEl.className = 'status-led yellow';
  } else {
    statusTextEl.textContent = '● Tracking Normally';
    statusLedEl.className = 'status-led green';
  }
}

function updateSliderDisplay() {
  ['wheelOdometry', 'lidar', 'gps'].forEach(name => {
    const el = document.getElementById(name + 'Accuracy');
    const valEl = document.getElementById(name + 'Value');
    if (el && valEl) valEl.textContent = parseFloat(el.value).toFixed(2);
  });
}

// ============================================================
// START / RESET
// ============================================================
function startSimulation() {
  if (running || completed || crashed) return;
  running = true;
  computeSlipParams();
  updateSensorTuning();

  document.getElementById('btnStart').disabled = true;
  document.getElementById('btnStart').textContent = '▶ RUNNING...';
  document.getElementById('btnStart').className = 'btn-start';
  document.getElementById('app').classList.add('locked');
}

function resetSimulation() {
  document.getElementById('resultPanel').style.display = 'none';
  document.getElementById('flashRed').classList.remove('show');

  initSimulation();

  document.getElementById('btnStart').disabled = false;
  document.getElementById('btnStart').textContent = '▶ DEPLOY AGV';
  document.getElementById('btnStart').className = 'btn-start ready';
  document.getElementById('app').classList.remove('locked');
}

// ============================================================
// UI EVENT BINDING (browser only)
// ============================================================
if (typeof document !== 'undefined') {
document.addEventListener('DOMContentLoaded', function () {
  const speedEl = document.getElementById('agvSpeed');
  if (speedEl) agvSpeed = parseFloat(speedEl.value);
  function setupSensorListener(name) {
    const accId = name + 'Accuracy';
    const valId = name + 'Value';
    const el = document.getElementById(accId);
    if (!el) return;
    el.addEventListener('input', function () {
      const valEl = document.getElementById(valId);
      if (valEl) valEl.textContent = parseFloat(this.value).toFixed(2);
      if (!running) {
        updateSensorTuning();
        updateMetrics();
      }
    });
    const toggleId = 'sensor' + name.charAt(0).toUpperCase() + name.slice(1);
    const toggleEl = document.getElementById(toggleId);
    if (toggleEl) {
      toggleEl.addEventListener('change', function () {
        if (!running) {
          updateSensorTuning();
          updateMetrics();
        }
      });
    }
  }
  setupSensorListener('wheelOdometry');
  setupSensorListener('lidar');
  setupSensorListener('gps');

  document.getElementById('agvSpeed').addEventListener('input', function () {
    agvSpeed = parseFloat(this.value);
    document.getElementById('speedValue').textContent = agvSpeed.toFixed(1);
    if (!running) {
      buildTrajectory();
    }
  });

  document.getElementById('btnAddBeacon').addEventListener('click', function () {
    if (running || completed || crashed) return;
    placingSensor = !placingSensor;
    this.textContent = placingSensor ? 'Click map to place...' : '+ Add Beacon';
  });
  renderExternalSensorList();

  document.querySelectorAll('.mode-btn').forEach(btn => {
    btn.addEventListener('click', function () {
      if (running) return;
      document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
      this.classList.add('active');
      slipMode = this.dataset.mode;
      const params = document.getElementById('corridorParams');
      if (params) {
        params.classList.toggle('show', slipMode === 'deterministic');
      }
      computeSlipParams();
    });
  });

  if (typeof renderMathInElement === 'function') {
    renderMathInElement(document.body, {
      delimiters: [
        { left: '$$', right: '$$', display: true },
        { left: '$', right: '$', display: false }
      ],
      throwOnError: false
    });
  }
});
}

// ============================================================
// NODE.JS EXPORTS (for testing)
// ============================================================
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    EKFSlam,
    cubicBezier,
    buildTrajectory,
    calculateSavings,
    measurementToPosition,
    isInCorridor,
    getCorruptedControl,
    noisyMeasurement,
    computeSlipParams,
    TOTAL_TIME,
    CORRIDOR_T_START,
    CORRIDOR_T_END,
    LIDAR_COST,
    MAX_SAVINGS_SIGMA
  };
}
