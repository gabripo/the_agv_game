const { expect } = require('chai');
const {
  cubicBezier,
  buildTrajectory,
  calculateSavings,
  measurementToPosition,
  isInCorridor,
  getCorruptedControl,
  noisyMeasurement,
  TOTAL_TIME,
  CORRIDOR_T_START,
  CORRIDOR_T_END,
  LIDAR_COST,
  MAX_SAVINGS_SIGMA
} = require('../script.js');
const { EKFSlam } = require('../script.js');

// ------------------------------------------------------------------
// CUBIC BEZIER
// ------------------------------------------------------------------
describe('cubicBezier', function () {
  const p0 = { x: 0, y: 0 };
  const p1 = { x: 50, y: 0 };
  const p2 = { x: 100, y: 50 };
  const p3 = { x: 150, y: 100 };

  it('returns p0 at t=0', function () {
    const pt = cubicBezier(p0, p1, p2, p3, 0);
    expect(pt.x).to.be.closeTo(p0.x, 1e-10);
    expect(pt.y).to.be.closeTo(p0.y, 1e-10);
  });

  it('returns p3 at t=1', function () {
    const pt = cubicBezier(p0, p1, p2, p3, 1);
    expect(pt.x).to.be.closeTo(p3.x, 1e-10);
    expect(pt.y).to.be.closeTo(p3.y, 1e-10);
  });

  it('interpolates smoothly at t=0.5', function () {
    const pt = cubicBezier(p0, p1, p2, p3, 0.5);
    // For this control point geometry: x = 75, y = 31.25
    expect(pt.x).to.be.closeTo(75, 0.5);
    expect(pt.y).to.be.closeTo(31.25, 0.5);
  });
});

// ------------------------------------------------------------------
// MEASUREMENT TO POSITION
// ------------------------------------------------------------------
describe('measurementToPosition', function () {
  it('returns null if measurement or landmark is null', function () {
    expect(measurementToPosition(null, { x: 0, y: 0 })).to.be.null;
    expect(measurementToPosition([1, 0], null)).to.be.null;
  });

  it('converts range-bearing to Cartesian coordinates', function () {
    const landmark = { x: 10, y: 10 };
    const measurement = [5, Math.PI / 4]; // range 5, bearing 45°
    const pos = measurementToPosition(measurement, landmark);
    // The measurement position is: landmark - range * [cos(bearing), sin(bearing)]
    const expectedX = 10 - 5 * Math.cos(Math.PI / 4);
    const expectedY = 10 - 5 * Math.sin(Math.PI / 4);
    expect(pos.x).to.be.closeTo(expectedX, 1e-10);
    expect(pos.y).to.be.closeTo(expectedY, 1e-10);
  });
});

// ------------------------------------------------------------------
// IS IN CORRIDOR
// ------------------------------------------------------------------
describe('isInCorridor', function () {
  it('returns true for times within the corridor window (default deterministic)', function () {
    // With default settings, slipStartTime = CORRIDOR_T_START
    expect(isInCorridor(CORRIDOR_T_START)).to.be.true;
    expect(isInCorridor((CORRIDOR_T_START + CORRIDOR_T_END) / 2)).to.be.true;
    expect(isInCorridor(CORRIDOR_T_END)).to.be.true;
  });

  it('returns false outside the corridor window', function () {
    expect(isInCorridor(0)).to.be.false;
    expect(isInCorridor(CORRIDOR_T_START - 1)).to.be.false;
    expect(isInCorridor(CORRIDOR_T_END + 1)).to.be.false;
  });
});

// ------------------------------------------------------------------
// CALCULATE SAVINGS
// ------------------------------------------------------------------
describe('calculateSavings', function () {
  let ekf;

  beforeEach(function () {
    ekf = new EKFSlam();
  });

  it('returns 0.40 when sigma is 0 (perfect estimate)', function () {
    ekf.P = math.multiply(math.identity(4), 1e-10);
    const savings = calculateSavings(ekf);
    expect(savings).to.be.closeTo(0.40, 1e-4);
  });

  it('returns 0 when sigma >= MAX_SAVINGS_SIGMA', function () {
    ekf.P = math.multiply(math.identity(4), MAX_SAVINGS_SIGMA * MAX_SAVINGS_SIGMA + 1);
    const savings = calculateSavings(ekf);
    expect(savings).to.be.closeTo(0, 1e-10);
  });

  it('is monotonically decreasing with increasing uncertainty', function () {
    const savings = [];
    for (let scale of [0.1, 0.5, 1, 2, 5, 10]) {
      ekf.P = math.multiply(math.identity(4), scale);
      savings.push(calculateSavings(ekf));
    }
    for (let i = 1; i < savings.length; i++) {
      expect(savings[i]).to.be.at.most(savings[i - 1] + 1e-10);
    }
  });

  it('returns a value in [0, 0.40] for any input', function () {
    const scales = [1e-6, 0.1, 0.5, 1, 2, 5, 10, 100];
    for (const scale of scales) {
      ekf.P = math.multiply(math.identity(4), scale);
      const s = calculateSavings(ekf);
      expect(s).to.be.at.least(0);
      expect(s).to.be.at.most(0.40 + 1e-10);
    }
  });
});

// ------------------------------------------------------------------
// GET CORRUPTED CONTROL
// ------------------------------------------------------------------
describe('getCorruptedControl', function () {
  before(function () {
    buildTrajectory();
  });

  it('returns an array of two numbers (thetaDot, a)', function () {
    const control = getCorruptedControl(0);
    expect(control).to.be.an('array');
    expect(control.length).to.equal(2);
    expect(control[0]).to.be.a('number');
    expect(control[1]).to.be.a('number');
  });

  it('acceleration component is always 0 (constant velocity model)', function () {
    for (let t of [0, 50, 100, 200, 300, 399]) {
      const control = getCorruptedControl(t);
      expect(control[1]).to.equal(0);
    }
  });
});

// ------------------------------------------------------------------
// NOISY MEASUREMENT
// ------------------------------------------------------------------
describe('noisyMeasurement', function () {
  let ekf;

  beforeEach(function () {
    ekf = new EKFSlam();
    // Set R to known values for deterministic noise bound testing
    ekf.setTuning(1, 1);
  });

  it('returns an array of two numbers (range, bearing)', function () {
    const meas = noisyMeasurement({ x: 0, y: 0, theta: 0 }, { x: 5, y: 0 }, ekf);
    expect(meas).to.be.an('array');
    expect(meas.length).to.equal(2);
    expect(meas[0]).to.be.a('number');
    expect(meas[1]).to.be.a('number');
  });

  it('range is close to the true distance', function () {
    const trueState = { x: 0, y: 0, theta: 0 };
    const landmark = { x: 10, y: 0 };
    const trueRange = 10;
    let sum = 0;
    const N = 50;
    for (let i = 0; i < N; i++) {
      sum += noisyMeasurement(trueState, landmark, ekf)[0];
    }
    const avg = sum / N;
    expect(avg).to.be.closeTo(trueRange, 1.0);
  });
});
