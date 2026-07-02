// Test setup — provides global.math from math.js (same API as the CDN version)
// so that script.js can be loaded in Node.js for testing.
const math = require('mathjs');
global.math = math;
