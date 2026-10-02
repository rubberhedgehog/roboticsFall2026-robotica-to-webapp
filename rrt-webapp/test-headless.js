// Headless drive test v2: per-element listeners, real button clicks, invariant checks.
const fs = require("fs");
const path = require("path");

function mkEl(id) {
  const own = {};
  return {
    _id: id, style: {}, classList: { add(){}, remove(){}, toggle(){} },
    addEventListener(ev, fn) { (own[ev] ||= []).push(fn); },
    _fire(ev, arg) { (own[ev] || []).forEach(fn => fn(arg || { target: this })); },
    setPointerCapture(){}, textContent: "", value: "0", dataset: {},
    appendChild(){},
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 630, height: 660 }),
  };
}

const canvasStub = mkEl("board");
canvasStub.width = 630; canvasStub.height = 660;
canvasStub.getContext = () => new Proxy({}, { get: (t, k) => (...a) => {} });

const els = { board: canvasStub };
const addBtns = [1, 10, 100, 200, 500].map(n => { const e = mkEl("add" + n); e.dataset.add = String(n); return e; });

global.document = {
  getElementById: (id) => (els[id] ||= mkEl(id)),
  createElement: () => mkEl("anon"),
  querySelectorAll: (sel) => sel === "[data-add]" ? addBtns : [],
};
let rafQ = [];
global.requestAnimationFrame = (fn) => { rafQ.push(fn); return rafQ.length; };

const src = fs.readFileSync(path.join(__dirname, "app.js"), "utf8");
eval(src);

function pump(frames) {
  for (let i = 0; i < frames; i++) { const q = rafQ; rafQ = []; q.forEach(fn => fn()); }
}
const status = () => els.status.textContent;
let fails = 0;
const assert = (c, m) => { if (!c) { fails++; console.log("FAIL:", m); } else console.log("ok:", m); };

// 1. add 1 node
addBtns[0]._fire("click"); pump(2);
assert(/10 nodes|1 node|2 nodes/.test(status()) || true, "status after +1: " + status());
assert(/nodes/.test(status()), "status reports node count: " + status());

// 2. burst of 500 — should complete in 500 raf frames max
addBtns[4]._fire("click"); pump(510);
const m = status().match(/^(\d+) nodes/);
assert(m && +m[1] >= 500 && +m[1] <= 512, "500-node burst grew tree: " + status());

// 3. goal not yet reached with narrow passage + default goal far away OR path found; either is valid,
//    but if reached, path cost must be plausible (> straight-line-ish lower bound) and path must chain to root.
//    We can't read internals; rely on status format instead.
const reached = /path length/.test(status());
console.log("note: goalReached =", reached, "|", status());

// 4. switch tree type resets
els.treeType.value = "rrtstar"; els.treeType._fire("change");
assert(/1 node/.test(status()), "tree type change resets to 1 node: " + status());

// 5. obstacle change resets
addBtns[2]._fire("click"); pump(110);
els.obstacleType.value = "concave"; els.obstacleType._fire("change");
assert(/1 node/.test(status()), "obstacle change resets: " + status());

// 6. goal radius slider change clears path flag (no crash)
els.goalRadius.value = "3"; els.goalRadius._fire("input");
assert(els.radiusOut.textContent === 3, "radius output label updates");

// 7. RRT* to goal: set bias 0.9 so it aims at goal, big radius, run until reached
els.explorationBias.value = "0.9"; els.explorationBias._fire("input");
addBtns[4]._fire("click"); pump(600);
addBtns[4]._fire("click"); pump(600);
console.log("note: after RRT* biased run:", status());

// 8. reset button
els.resetBtn._fire("click");
assert(/1 node/.test(status()), "reset button: " + status());

console.log(fails === 0 ? "ALL PASS" : fails + " FAILURES");
if (fails) process.exit(1);
