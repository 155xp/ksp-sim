// Fly one landing through the UI in headless Chromium and screenshot it.
//   node landing.mjs [url] [seed] [outdir] [speedIndex]
// Prints the miss the UI shows and the server's done message as JSON.
import { chromium } from "playwright";
import { mkdirSync } from "node:fs";

const [url = "http://127.0.0.1:8787/", seed = "7", out = "../docs", speedIdx = "6"] = process.argv.slice(2);
mkdirSync(out, { recursive: true });

const browser = await chromium.launch({ args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"] });
const page = await browser.newPage({ viewport: { width: 1440, height: 900 }, deviceScaleFactor: 1 });
const errors = [];
page.on("pageerror", (e) => errors.push(String(e)));
page.on("console", (m) => m.type() === "error" && errors.push(m.text()));
await page.goto(url);
await page.waitForFunction(() => window.__viewer);
await page.screenshot({ path: `${out}/idle.png` });

await page.fill("#seed", seed);
await page.$eval("#speed", (el, v) => { el.value = v; el.dispatchEvent(new Event("input")); }, speedIdx);
await page.click("#run");
await page.waitForFunction(() => window.__viewer.rows > 0, null, { timeout: 60000 });

const phase = () => page.textContent("#tPhase");
async function shotAt(name, label, pause) {
  await page.waitForFunction((l) => document.getElementById("tPhase").textContent === l, label, { timeout: 600000, polling: 50 });
  if (pause) {
    await page.click("#play");
    await page.waitForTimeout(400);
  }
  await page.screenshot({ path: `${out}/${name}.png` });
  const t = await page.textContent("#clock");
  if (pause) await page.click("#play");
  return t;
}
const mid = await shotAt("mid-flight-boostback", "Boostback", true);
const burn = await shotAt("mid-flight-landing-burn", "Landing burn", true);
await page.waitForFunction(() => window.__viewer.mode === "finished", null, { timeout: 600000 });
await page.waitForTimeout(500);
await page.screenshot({ path: `${out}/touchdown.png` });

// orbit camera, zoomed out over the whole flight
await page.click('.seg[data-name="camera"] button[data-v="orbit"]');
await page.mouse.move(720, 450);
for (let i = 0; i < 34; i++) { await page.mouse.wheel(0, 400); await page.waitForTimeout(30); }
await page.mouse.move(720, 450);
await page.mouse.down();
await page.mouse.move(560, 420, { steps: 10 });
await page.mouse.up();
await page.waitForTimeout(800);
await page.screenshot({ path: `${out}/orbit-overview.png` });

await page.click('.seg[data-name="camera"] button[data-v="follow"]');

// replay: scrub back to the middle and check the readouts follow
await page.$eval("#scrub", (el) => { el.value = String(parseFloat(el.max) / 2); el.dispatchEvent(new Event("input")); });
await page.waitForTimeout(300);
await page.screenshot({ path: `${out}/scrubbed.png` });
const scrubbed = { clock: await page.textContent("#clock"), phase: await phase() };

// replay from the scrubbed point, then reset
const before = await page.evaluate(() => window.__viewer.tDisp);
await page.click("#play");
await page.waitForTimeout(1500);
const replay = { mode: await page.evaluate(() => window.__viewer.mode), advanced_s: (await page.evaluate(() => window.__viewer.tDisp)) - before };

const ui = {
  outcome: await page.textContent("#outcome"),
  miss: await page.textContent("#rMiss"),
  speed: await page.textContent("#rSpeed"),
  design: await page.textContent("#design"),
};
const done = await page.evaluate(() => window.__viewer.result);
await page.click("#reset");
await page.waitForTimeout(300);
const afterReset = await page.evaluate(() => window.__viewer.mode);
console.log(JSON.stringify({ seed: +seed, ui, mid_clock: mid, burn_clock: burn, scrubbed, replay, afterReset, server: done, errors }, null, 1));
await browser.close();
