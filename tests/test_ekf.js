const { expect } = require('chai');
const { EKFSlam } = require('../script.js');

describe('EKFSlam', function () {
  let ekf;

  beforeEach(function () {
    ekf = new EKFSlam();
  });

  // ------------------------------------------------------------------
  // CONSTRUCTOR
  // ------------------------------------------------------------------
  describe('constructor', function () {
    it('initialises state as a 4×1 zero matrix', function () {
      const s = ekf.state.valueOf();
      expect(s).to.be.an('array');
      expect(s.length).to.equal(4);
      s.forEach(row => expect(row.length).to.equal(1));
      expect(s[0][0]).to.equal(0);
      expect(s[1][0]).to.equal(0);
      expect(s[2][0]).to.equal(0);
      expect(s[3][0]).to.equal(0);
    });

    it('initialises P as a 4×4 identity × 0.1', function () {
      const p = ekf.P.valueOf();
      expect(p.length).to.equal(4);
      p.forEach(row => expect(row.length).to.equal(4));
      const expected = math.multiply(math.identity(4), 0.1).valueOf();
      for (let i = 0; i < 4; i++)
        for (let j = 0; j < 4; j++)
          expect(p[i][j]).to.be.closeTo(expected[i][j], 1e-10);
    });

    it('initialises Q as a 4×4 diagonal matrix with default values', function () {
      const q = ekf.Q.valueOf();
      expect(q[0][0]).to.be.closeTo(0.5, 1e-10);
      expect(q[1][1]).to.be.closeTo(0.5, 1e-10);
      expect(q[2][2]).to.be.closeTo(0.1, 1e-10);
      expect(q[3][3]).to.be.closeTo(0.05, 1e-10);
    });

    it('initialises R as a 2×2 diagonal matrix with default values', function () {
      const r = ekf.R.valueOf();
      expect(r[0][0]).to.be.closeTo(0.5, 1e-10);
      expect(r[1][1]).to.be.closeTo(0.5, 1e-10);
    });
  });

  // ------------------------------------------------------------------
  // STATE ACCESSORS
  // ------------------------------------------------------------------
  describe('state accessors', function () {
    it('setState and getters return correct values', function () {
      ekf.setState(10, 20, 0.5, 2.0);
      expect(ekf.getX()).to.be.closeTo(10, 1e-10);
      expect(ekf.getY()).to.be.closeTo(20, 1e-10);
      expect(ekf.getTheta()).to.be.closeTo(0.5, 1e-10);
      expect(ekf.getV()).to.be.closeTo(2.0, 1e-10);
    });
  });

  // ------------------------------------------------------------------
  // NORMALISE ANGLE
  // ------------------------------------------------------------------
  describe('normalizeAngle', function () {
    it('returns angles in [-π, π]', function () {
      const testAngles = [0, Math.PI, -Math.PI, 3.5, -4.2, 100, -100];
      for (const a of testAngles) {
        const n = ekf.normalizeAngle(a);
        expect(n).to.be.at.least(-Math.PI);
        expect(n).to.be.at.most(Math.PI);
      }
    });

    it('preserves angles already in [-π, π]', function () {
      expect(ekf.normalizeAngle(0)).to.be.closeTo(0, 1e-10);
      expect(ekf.normalizeAngle(1.5)).to.be.closeTo(1.5, 1e-10);
      expect(ekf.normalizeAngle(-2.0)).to.be.closeTo(-2.0, 1e-10);
    });

    it('wraps around multiples of 2π', function () {
      expect(ekf.normalizeAngle(2 * Math.PI)).to.be.closeTo(0, 1e-10);
      expect(ekf.normalizeAngle(3 * Math.PI)).to.be.closeTo(Math.PI, 1e-8);
      expect(ekf.normalizeAngle(-3 * Math.PI)).to.be.closeTo(-Math.PI, 1e-8);
    });
  });

  // ------------------------------------------------------------------
  // MOTION MODEL f()
  // ------------------------------------------------------------------
  describe('f — motion model', function () {
    it('advances state according to kinematic bicycle model', function () {
      ekf.setState(0, 0, 0, 1); // x=0, y=0, theta=0, v=1
      const result = ekf.f(ekf.state, [0, 0], 1); // no turn, no accel
      const r = result.valueOf();
      expect(r[0][0]).to.be.closeTo(1, 1e-6);  // x += v*cos(0)*1 = 1
      expect(r[1][0]).to.be.closeTo(0, 1e-6);  // y += v*sin(0)*1 = 0
      expect(r[2][0]).to.be.closeTo(0, 1e-6);  // theta unchanged
      expect(r[3][0]).to.be.closeTo(1, 1e-6);  // v unchanged
    });

    it('applies steering correctly', function () {
      ekf.setState(0, 0, 0, 1);
      const result = ekf.f(ekf.state, [0.1, 0], 1);
      const r = result.valueOf();
      expect(r[2][0]).to.be.closeTo(0.1, 1e-6); // theta += thetaDot * dt
    });

    it('applies acceleration', function () {
      ekf.setState(0, 0, 0, 1);
      const result = ekf.f(ekf.state, [0, 0.5], 1);
      const r = result.valueOf();
      expect(r[3][0]).to.be.closeTo(1.5, 1e-6); // v += a * dt
    });

    it('moves diagonally when heading is non-zero', function () {
      ekf.setState(0, 0, Math.PI / 4, 1);
      const result = ekf.f(ekf.state, [0, 0], 1);
      const r = result.valueOf();
      const s = Math.SQRT1_2; // sin(45°) = cos(45°) = √2/2
      expect(r[0][0]).to.be.closeTo(s, 1e-6);
      expect(r[1][0]).to.be.closeTo(s, 1e-6);
    });
  });

  // ------------------------------------------------------------------
  // JACOBIAN F
  // ------------------------------------------------------------------
  describe('jacobianF', function () {
    it('returns a 4×4 matrix', function () {
      ekf.setState(0, 0, 0, 1);
      const F = ekf.jacobianF(ekf.state, [0, 0], 1);
      const fv = F.valueOf();
      expect(fv.length).to.equal(4);
      fv.forEach(row => expect(row.length).to.equal(4));
    });

    it('has expected structure for straight motion', function () {
      ekf.setState(0, 0, 0, 2);
      const F = ekf.jacobianF(ekf.state, [0, 0], 1);
      const fv = F.valueOf();
      // F[0,2] = -v * sin(theta) * dt = -2 * 0 * 1 = 0
      expect(fv[0][2]).to.be.closeTo(0, 1e-10);
      // F[0,3] = cos(theta) * dt = cos(0) * 1 = 1
      expect(fv[0][3]).to.be.closeTo(1, 1e-10);
      // F[1,2] = v * cos(theta) * dt = 2 * 1 * 1 = 2
      expect(fv[1][2]).to.be.closeTo(2, 1e-10);
      // F[2,2] = 1
      expect(fv[2][2]).to.be.closeTo(1, 1e-10);
      // F[3,3] = 1
      expect(fv[3][3]).to.be.closeTo(1, 1e-10);
    });
  });

  // ------------------------------------------------------------------
  // PREDICT
  // ------------------------------------------------------------------
  describe('predict', function () {
    it('increases position uncertainty (trace of P grows)', function () {
      const before = ekf.getPositionUncertainty();
      ekf.predict([0.1, 0], 1);
      const after = ekf.getPositionUncertainty();
      expect(after).to.be.at.least(before);
    });

    it('modifies the state', function () {
      ekf.setState(0, 0, 0, 1);
      ekf.predict([0, 0], 1);
      expect(ekf.getX()).to.be.closeTo(1, 1e-6);
    });

    it('normalises theta after prediction', function () {
      ekf.setState(0, 0, Math.PI, 0);
      ekf.predict([Math.PI, 0], 2); // theta += 2π
      expect(ekf.getTheta()).to.be.closeTo(Math.PI, 1e-6);
    });
  });

  // ------------------------------------------------------------------
  // MEASUREMENT MODEL h()
  // ------------------------------------------------------------------
  describe('h — measurement model', function () {
    it('returns range and bearing to a landmark', function () {
      ekf.setState(0, 0, 0, 0);
      const lm = { x: 3, y: 4 };
      const [range, bearing] = ekf.h(ekf.state, lm);
      expect(range).to.be.closeTo(5, 1e-6); // 3-4-5 triangle
      expect(bearing).to.be.closeTo(Math.atan2(4, 3), 1e-6);
    });

    it('bearing is relative to heading', function () {
      ekf.setState(0, 0, Math.PI / 2, 0); // facing north
      const lm = { x: 3, y: 0 };
      const [range, bearing] = ekf.h(ekf.state, lm);
      // Landmark is at bearing -90° relative to heading
      expect(bearing).to.be.closeTo(-Math.PI / 2, 1e-6);
    });
  });

  // ------------------------------------------------------------------
  // JACOBIAN H
  // ------------------------------------------------------------------
  describe('jacobianH', function () {
    it('returns a 2×4 matrix', function () {
      ekf.setState(5, 5, 0, 0);
      const H = ekf.jacobianH(ekf.state, { x: 10, y: 10 });
      const hv = H.valueOf();
      expect(hv.length).to.equal(2);
      hv.forEach(row => expect(row.length).to.equal(4));
    });

    it('returns zeros for degenerate range (robot on landmark)', function () {
      ekf.setState(10, 10, 0, 0);
      const H = ekf.jacobianH(ekf.state, { x: 10, y: 10 });
      const hv = H.valueOf();
      expect(hv[0][0]).to.equal(0);
      expect(hv[1][0]).to.equal(0);
    });
  });

  // ------------------------------------------------------------------
  // UPDATE
  // ------------------------------------------------------------------
  describe('update', function () {
    it('reduces position uncertainty (trace of P shrinks)', function () {
      ekf.setState(0, 0, 0, 0);
      ekf.predict([0, 0], 1); // grow P
      const before = ekf.getPositionUncertainty();
      const lm = { x: 5, y: 0 };
      const meas = [5, 0]; // perfect measurement
      ekf.update(meas, lm);
      const after = ekf.getPositionUncertainty();
      expect(after).to.be.at.most(before);
    });

    it('converges state toward truth with repeated correct measurements', function () {
      // Start with incorrect estimate far from origin
      ekf.setState(10, 10, 0, 0);
      const lm = { x: 5, y: 0 };

      for (let i = 0; i < 30; i++) {
        ekf.predict([0, 0], 0.1);
        // Perfect measurement from origin to landmark at (5,0)
        const range = 5;
        const bearing = Math.atan2(0 - 0, 5 - 0);
        ekf.update([range, bearing], lm);
      }

      // State should have moved significantly toward origin
      const dist = Math.sqrt(ekf.getX() ** 2 + ekf.getY() ** 2);
      expect(dist).to.be.lessThan(9);
    });
  });

  // ------------------------------------------------------------------
  // SET TUNING
  // ------------------------------------------------------------------
  describe('setTuning', function () {
    it('scales Q by the given factor', function () {
      ekf.setTuning(2, 1);
      const q = ekf.Q.valueOf();
      expect(q[0][0]).to.be.closeTo(1.0, 1e-10);
      expect(q[1][1]).to.be.closeTo(1.0, 1e-10);
    });

    it('scales R by the given factor', function () {
      ekf.setTuning(1, 3);
      const r = ekf.R.valueOf();
      expect(r[0][0]).to.be.closeTo(1.5, 1e-10);
      expect(r[1][1]).to.be.closeTo(1.5, 1e-10);
    });

    it('does not mutate the default matrices', function () {
      const origQ = math.clone(ekf.defaultQ);
      ekf.setTuning(5, 5);
      const dq = ekf.defaultQ.valueOf();
      expect(dq[0][0]).to.be.closeTo(origQ.valueOf()[0][0], 1e-10);
    });
  });

  // ------------------------------------------------------------------
  // POSITION UNCERTAINTY
  // ------------------------------------------------------------------
  describe('getPositionUncertainty', function () {
    it('returns trace of the 2×2 position sub-block of P', function () {
      const p = ekf.P.valueOf();
      const expected = p[0][0] + p[1][1];
      expect(ekf.getPositionUncertainty()).to.be.closeTo(expected, 1e-10);
    });

    it('returns a non-negative value', function () {
      expect(ekf.getPositionUncertainty()).to.be.at.least(0);
      ekf.predict([0.1, 0], 1);
      expect(ekf.getPositionUncertainty()).to.be.at.least(0);
    });
  });

  describe('getPositionSigma', function () {
    it('returns sqrt of uncertainty', function () {
      const u = ekf.getPositionUncertainty();
      expect(ekf.getPositionSigma()).to.be.closeTo(Math.sqrt(u), 1e-10);
    });
  });

  // ------------------------------------------------------------------
  // INTEGRATION: FULL PREDICT-UPDATE CYCLE
  // ------------------------------------------------------------------
  describe('full predict-update cycle', function () {
    it('runs 50 steps without numerical instability', function () {
      ekf.setState(150, 440, 0, 2);
      const lm = { x: 300, y: 480 };

      for (let i = 0; i < 50; i++) {
        ekf.predict([0.02, 0], 1);
        const truePos = { x: 150 + i * 2, y: 440 };
        const dx = lm.x - truePos.x;
        const dy = lm.y - truePos.y;
        const range = Math.sqrt(dx * dx + dy * dy) + (Math.random() - 0.5) * 0.5;
        const bearing = Math.atan2(dy, dx) - 0.2 + (Math.random() - 0.5) * 0.02;
        ekf.update([range, bearing], lm);
      }

      // P should be finite and positive semi-definite
      const p = ekf.P.valueOf();
      expect(p[0][0]).to.be.finite;
      expect(p[0][0]).to.be.at.least(0);
      // State should be finite
      expect(ekf.getX()).to.be.finite;
      expect(ekf.getY()).to.be.finite;
    });
  });
});
