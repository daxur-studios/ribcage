/**
 * Purpose: the craft page, in a real (headless) browser, after the scripted hop with sleeping on
 * shows the same state hash as the same run in Node, with no errors on the page.
 * Value: plan 005, phase 5.12. The page only drives the core through commands, so the hash must match
 * the headless one; this proves it end to end (wasm, page code, renderer). It skips when there is no
 * browser or no built page (`pnpm build:web`).
 */
import assert from "node:assert/strict";
import { execFile } from "node:child_process";
import { existsSync, readFileSync } from "node:fs";
import { createServer } from "node:http";
import { extname, join, normalize } from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";
import { CommandBatch } from "../src/commands.ts";
import { Core } from "../src/core.ts";
import { scenarioFor } from "../src/craft-page.ts";
import { HOP_STEPS } from "../src/landing.ts";
import { World } from "../src/world.ts";

const DIST = fileURLToPath(new URL("../dist/", import.meta.url));
const WASM_PATH = new URL("../target/wasm32-unknown-unknown/release/beam_core.wasm", import.meta.url);
const HOPPER = readFileSync(new URL("./fixtures/hopper.craft.json", import.meta.url), "utf8");

const BROWSERS = [
  process.env["BROWSER_PATH"] ?? "",
  "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe",
  "C:/Program Files/Microsoft/Edge/Application/msedge.exe",
  "C:/Program Files/Google/Chrome/Application/chrome.exe",
  "/usr/bin/google-chrome",
  "/usr/bin/chromium",
];
const browser = BROWSERS.find((p) => p !== "" && existsSync(p));
const MIME: Record<string, string> = { ".html": "text/html", ".js": "text/javascript", ".json": "application/json", ".wasm": "application/wasm", ".rcrd": "application/octet-stream" };

async function nodeHash(): Promise<string> {
  const core = await Core.load(new Uint8Array(readFileSync(WASM_PATH)));
  const sc = scenarioFor("hop", HOPPER, { sleeping: true });
  const world = new World(core);
  world.apply(sc.flight.setup);
  for (let s = 0; s < HOP_STEPS; s++) {
    const b = new CommandBatch();
    sc.script!(b, s);
    if (b.count > 0) world.apply(b);
    world.step();
  }
  const hash = world.stateHash;
  world.free();
  return `0x${hash.toString(16).padStart(16, "0")}`;
}

test("the page's scripted hop with sleeping on shows Node's hash and no errors", { skip: browser === undefined || !existsSync(join(DIST, "web", "craft.html")) ? "no browser or no built page (pnpm build:web)" : false, timeout: 240_000 }, async () => {
  const server = createServer((req, res) => {
    const path = normalize(join(DIST, decodeURIComponent((req.url ?? "/").split("?")[0]!)));
    if (!path.startsWith(DIST) || !existsSync(path)) {
      res.statusCode = 404;
      res.end();
      return;
    }
    res.setHeader("content-type", MIME[extname(path)] ?? "application/octet-stream");
    res.end(readFileSync(path));
  });
  await new Promise<void>((ok) => server.listen(0, "127.0.0.1", ok));
  const port = (server.address() as { port: number }).port;
  try {
    const url = `http://127.0.0.1:${port}/web/craft.html?scene=hop&steps=${HOP_STEPS}&stopat=${HOP_STEPS}&jump=1&webgl=1&sleep=1`;
    let dom = "";
    // Software rendering in a headless browser now and then stalls; a stalled run is killed and tried again.
    for (let attempt = 0; attempt < 5 && !dom.includes("hash 0x"); attempt++) {
      dom = await new Promise<string>((done) => {
        const child = execFile(
          browser!,
          ["--headless=new", "--disable-gpu", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--no-sandbox", "--virtual-time-budget=3000", "--dump-dom", url],
          { encoding: "utf8", timeout: 25_000, maxBuffer: 1 << 26 },
          (_e, out) => done(out ?? ""),
        );
        child.on("error", () => done(""));
      });
    }
    const m = /hash (0x[0-9a-f]{16})/.exec(dom);
    assert.ok(m, `the page did not print a hash: ${dom.slice(0, 300)}`);
    assert.equal(m![1], await nodeHash());
    assert.match(dom, /sleeping on/);
    assert.match(dom, /data-errors="0"/);
  } finally {
    server.close();
  }
});
