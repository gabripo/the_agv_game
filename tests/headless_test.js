const { chromium, firefox } = require('playwright');
const path = require('path');
const fs = require('fs');

const SCREENSHOT_DIR = path.join(__dirname, '..', 'screenshots');

async function run() {
  fs.mkdirSync(SCREENSHOT_DIR, { recursive: true });

  // Use Firefox as specified by the user
  const browser = await firefox.launch({ headless: true });
  const context = await browser.newContext({
    viewport: { width: 1400, height: 900 },
    ignoreHTTPSErrors: true,
  });

  const page = await context.newPage();
  const errors = [];

  // Collect all console messages
  page.on('console', msg => {
    if (msg.type() === 'error') {
      errors.push({ type: 'console.error', text: msg.text(), location: msg.location() });
      console.error('  ❌ CONSOLE ERROR:', msg.text());
    }
  });

  // Collect uncaught exceptions
  page.on('pageerror', err => {
    errors.push({ type: 'pageerror', text: err.message, stack: err.stack });
    console.error('  ❌ PAGE ERROR:', err.message);
  });

  // Collect request failures (e.g. CDN loading)
  page.on('requestfailed', req => {
    errors.push({ type: 'requestfailed', url: req.url(), status: req.failure() });
    console.error('  ❌ REQUEST FAILED:', req.url(), req.failure()?.errorText);
  });

  const filePath = 'file://' + path.join(__dirname, '..', 'index.html');
  console.log('Opening:', filePath);

  await page.goto(filePath, { waitUntil: 'networkidle', timeout: 30000 });
  console.log('Page loaded, waiting for p5.js initialization...');

  // Wait a bit for p5.js to initialise
  await page.waitForTimeout(2000);

  // Screenshot after load
  await page.screenshot({ path: path.join(SCREENSHOT_DIR, '01-initial.png'), fullPage: true });
  console.log('Screenshot: 01-initial.png');

  // Check if canvas exists
  const canvasCount = await page.evaluate(() => document.querySelectorAll('canvas').length);
  console.log(`Canvas elements found: ${canvasCount}`);

  const canvasVisible = await page.evaluate(() => {
    const c = document.querySelector('canvas');
    if (!c) return false;
    const rect = c.getBoundingClientRect();
    return rect.width > 0 && rect.height > 0;
  });
  console.log(`Canvas visible: ${canvasVisible}`);

  // Check p5.js functions exist
  const p5Exists = await page.evaluate(() => {
    return typeof setup === 'function' && typeof draw === 'function';
  });
  console.log(`p5.js setup/draw defined: ${p5Exists}`);

  // Check if the canvas has been drawn (non-black pixels)
  const canvasHasContent = await page.evaluate(() => {
    const c = document.querySelector('canvas');
    if (!c) return false;
    const ctx = c.getContext('2d');
    if (!ctx) return false;
    const img = ctx.getImageData(0, 0, c.width, c.height);
    const d = img.data;
    let sum = 0;
    for (let i = 0; i < d.length; i += 4) {
      sum += d[i] + d[i+1] + d[i+2];
    }
    return sum > 0;
  });
  console.log(`Canvas has drawn content: ${canvasHasContent}`);

  // Check for the idle prompt
  const idleText = await page.evaluate(() => {
    const texts = [];
    document.querySelectorAll('canvas').forEach(c => {
      texts.push(c.width, c.height);
    });
    return texts;
  });
  console.log(`Canvas dimensions: ${idleText}`);

  // Check slider values
  const sliderValues = await page.evaluate(() => {
    const q = document.getElementById('qSlider')?.value;
    const r = document.getElementById('rSlider')?.value;
    return { q, r };
  });
  console.log(`Slider values: Q=${sliderValues.q}, R=${sliderValues.r}`);

  // Check mode selector
  const activeMode = await page.evaluate(() => {
    const active = document.querySelector('.mode-btn.active');
    return active ? active.dataset.mode : 'none';
  });
  console.log(`Active mode: ${activeMode}`);

  // Check start button state
  const btnState = await page.evaluate(() => {
    const btn = document.getElementById('btnStart');
    return {
      disabled: btn.disabled,
      text: btn.textContent,
      className: btn.className
    };
  });
  console.log(`Start button: disabled=${btnState.disabled}, text="${btnState.text.trim()}"`);

  // Try to click the start button
  console.log('Clicking DEPLOY AGV button...');
  const clickResult = await page.evaluate(() => {
    try {
      const btn = document.getElementById('btnStart');
      btn.click();
      return { success: true, running: typeof running !== 'undefined' ? running : 'undefined' };
    } catch (e) {
      return { success: false, error: e.message };
    }
  });
  console.log(`Click result:`, clickResult);

  await page.waitForTimeout(500);

  // Check if running state changed
  const afterClick = await page.evaluate(() => {
    return {
      running,
      simTime,
      btnText: document.getElementById('btnStart')?.textContent?.trim()
    };
  });
  console.log(`After click: running=${afterClick.running}, simTime=${afterClick.simTime}, button="${afterClick.btnText}"`);

  // Screenshot after clicking
  await page.screenshot({ path: path.join(SCREENSHOT_DIR, '02-after-click.png'), fullPage: true });
  console.log('Screenshot: 02-after-click.png');

  // Wait and check if simulation progresses
  await page.waitForTimeout(2000);
  const simState = await page.evaluate(() => ({
    simTime,
    running,
    completed,
    crashed,
    ekfX: typeof ekf !== 'undefined' ? ekf.getX() : 'undefined',
    ekfY: typeof ekf !== 'undefined' ? ekf.getY() : 'undefined',
  }));
  console.log(`Simulation state after 2s:`, simState);

  // Screenshot during simulation
  await page.screenshot({ path: path.join(SCREENSHOT_DIR, '03-during-sim.png'), fullPage: true });

  // Wait for simulation to complete
  await page.waitForTimeout(8000);

  const finalState = await page.evaluate(() => ({
    simTime,
    running,
    completed,
    crashed,
    maxDivergence
  }));
  console.log(`Final state:`, finalState);

  await page.screenshot({ path: path.join(SCREENSHOT_DIR, '04-final.png'), fullPage: true });
  console.log('Screenshot: 04-final.png');

  // Summary
  console.log('\n=== TEST SUMMARY ===');
  console.log(`Errors found: ${errors.length}`);
  if (errors.length > 0) {
    console.log('\nAll errors:');
    errors.forEach((e, i) => {
      console.log(`  ${i + 1}. [${e.type}] ${e.text}`);
      if (e.stack) console.log(`     ${e.stack.split('\n')[1]}`);
    });
  }

  const success = errors.length === 0 && simState.running === true;
  console.log(`\nTest ${success ? 'PASSED' : 'FAILED'}`);

  await browser.close();
  return success;
}

run().then(success => {
  process.exit(success ? 0 : 1);
}).catch(err => {
  console.error('Test harness error:', err.message);
  process.exit(1);
});
