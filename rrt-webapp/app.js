/*
 * RRT / RRT* demonstration — JS port of the Wolfram Manipulate
 * "Rapidly Exploring Random Tree (RRT) and RRT*" by Aaron T. Becker & Li Huang.
 *
 * Faithful port notes:
 *  - verts entry: [ {x,y}, parent(1-based, 0=root), index, cost, children[] ]
 *  - step size 1, RRT* neighborhood radius rrtDist = 3
 *  - goal locator clamped to [-19,19]; PlotRange {{-21,21},{-21,23}}
 *  - the original's burst behaviour is kept: while pending > 5 nodes are added
 *    without redraw, then the last few animate one per frame.
 */
(() => {
"use strict";

// ---------- geometry helpers (exact ports of the Initialization cell) ----------

// angtest[p1,p2] = p1 . {{0,-1},{1,0}} . p2 > 0  ==  y1*x2 - x1*y2 > 0
function angtest(p1, p2) {
  return p1[1] * p2[0] - p1[0] * p2[1] > 0;
}

// ptInPoly: all angtest(v_i - pt, v_{i+1} - pt) must be Equal (to the first one).
// In the notebook: Apply[Equal,#]& on the list of booleans → True only if all equal.
// The first element decides the required value.
function ptInPoly(poly, pt) {
  const rel = poly.map(v => [v[0] - pt[0], v[1] - pt[1]]);
  const res = [];
  for (let i = 0; i < rel.length; i++) res.push(angtest(rel[i], rel[(i + 1) % rel.length]));
  if (res.length === 0) return false;
  return res.every(b => res[0] === b);
}

// lineIntersectionPoint[{{a,b},{c,d}}]
function lineIntersectionPoint(a, b, c, d) {
  const det = (u, v) => u[0] * v[1] - u[1] * v[0];
  const denom = det([a[0] - b[0], a[1] - b[1], 0].slice(0, 2), [c[0] - d[0], c[1] - d[1]]);
  const cab = det(a, b), ccd = det(c, d);
  const da = [a[0] - b[0], a[1] - b[1]], dc = [c[0] - d[0], c[1] - d[1]];
  return [
    (cab * dc[0] - ccd * da[0]) / denom,
    (cab * dc[1] - ccd * da[1]) / denom,
  ];
}

// lambda[{{a,b}}][p] = (a-p).(a-b) / (a-b).(a-b)
function lambda(a, b, p) {
  const ab = [a[0] - b[0], a[1] - b[1]];
  const ap = [a[0] - p[0], a[1] - p[1]];
  return (ap[0] * ab[0] + ap[1] * ab[1]) / (ab[0] * ab[0] + ab[1] * ab[1]);
}

// segmentIntersectionQ[{ {a,b}, {c,d} }]
function segmentIntersectionQ(a, b, c, d) {
  const det = (u, v) => u[0] * v[1] - u[1] * v[0];
  if (det([a[0] - b[0], a[1] - b[1]], [c[0] - d[0], c[1] - d[1]]) === 0) return false;
  const p = lineIntersectionPoint(a, b, c, d);
  const l1 = lambda(a, b, p), l2 = lambda(c, d, p);
  return 0 <= l1 && l1 <= 1 && 0 <= l2 && l2 <= 1;
}

// inCollision[nv, np, obsedges]: np inside a polygon, or segment nv-np crosses an edge.
function inCollision(nv, np, obsedges) {
  let collision = false;
  for (let i = 0; i < obsedges.length && collision !== true; i++) {
    if (ptInPoly(obsedges[i], np)) collision = true;
  }
  for (let i = 0; i < obsedges.length && collision !== true; i++) {
    const poly = obsedges[i];
    for (let j = 0; j < poly.length; j++) {
      const p1 = poly[j], p2 = poly[(j + 1) % poly.length];
      if (segmentIntersectionQ(nv, np, p1, p2)) { collision = true; break; }
    }
  }
  return collision;
}

// ---------- obstacle sets (verbatim from the notebook) ----------

const OBSTACLES = {
  "none": [],
  "sparse": [
    [[1, 1], [1, 17], [17, 17], [17, 1]],
    [[-1, 1], [-1, 17], [-17, 17], [-17, 1]],
    [[1, -1], [1, -17], [17, -17], [17, -1]],
    // -{{1,1},{1,17},{17,17},{17,1}}  (Mathematica negates each coordinate)
    [[-1, -1], [-1, -17], [-17, -17], [-17, -1]],
  ],
  "narrow": [
    [[5, 10], [6, 10], [6, 21], [5, 21]],
    [[5, 5], [6, 5], [6, -21], [5, -21]],
    [[-5, -7], [-6, -7], [-6, 21], [-5, 21]],
    [[-5, -9], [-6, -9], [-6, -21], [-5, -21]],
  ],
  "concave": [
    [[5, 3], [-6, 3], [-6, 4], [6, 4], [6, -4], [-6, -4], [-6, -3], [5, -3]],
  ],
};

// ---------- state ----------

const state = {
  treeType: "rrt",          // "random" | "rrt" | "rrtstar"
  obstacleType: "narrow",
  explorationBias: 0,
  goalRadius: 1,
  goal: [18, -18],
  verts: [{ pos: [0.0, 0.0], parent: 0, index: 1, cost: 0, children: [] }],
  path: [],
  reachGoal: false,
  newrand: null,           // last sampled sample point p
  pending: 0,              // nodestoadd
};
const RRT_DIST = 3;
const STEP = 1;

function resetTree() {
  state.verts = [{ pos: [0.0, 0.0], parent: 0, index: 1, cost: 0, children: [] }];
  state.path = [];
  state.reachGoal = false;
  state.newrand = null;
  state.pending = 0;
  stopAuto();
}

// ---------- core algorithm ----------

function nearestIndex(p) {
  let best = 0, bestD = Infinity;
  for (let i = 0; i < state.verts.length; i++) {
    const v = state.verts[i].pos;
    const d = Math.hypot(v[0] - p[0], v[1] - p[1]);
    if (d < bestD) { bestD = d; best = i; }
  }
  return best;
}

function dist(a, b) { return Math.hypot(a[0] - b[0], a[1] - b[1]); }

// addNode — port of addNode$[treeType, obsedges]
function addNode(obsedges) {
  const isRRT = state.treeType === "rrt" || state.treeType === "rrtstar";
  const isRRTStar = state.treeType === "rrtstar";
  let success = false, p, nvPos, parent, np, nvs = [], eucDist = [];

  while (!success) {
    if (Math.random() < state.explorationBias) {
      p = state.goal.slice();
    } else {
      p = [Math.round((Math.random() * 40 - 20) * 100) / 100,
           Math.round((Math.random() * 40 - 20) * 100) / 100];
    }

    if (isRRT) {
      nvPos = state.verts[nearestIndex(p)].pos;
    } else {
      nvPos = state.verts[Math.floor(Math.random() * state.verts.length)].pos;
    }
    parent = nearestIndex(nvPos); // Position of nv in verts (0-based here)

    const diffv = [p[0] - nvPos[0], p[1] - nvPos[1]];
    const dn = Math.hypot(diffv[0], diffv[1]);
    np = dn > STEP
      ? [nvPos[0] + STEP * diffv[0] / dn, nvPos[1] + STEP * diffv[1] / dn]
      : p.slice();

    if (isRRTStar) {
      // nvs$ = all verts within rrtDist of np
      nvs = [];
      eucDist = [];
      for (let i = 0; i < state.verts.length; i++) {
        const vs = state.verts[i].pos;
        if (dist(vs, np) <= RRT_DIST) nvs.push(i);
      }
      for (const i of nvs) {
        const vs = state.verts[i].pos;
        eucDist.push(inCollision(vs, np, obsedges) ? Infinity : dist(vs, np));
      }
      const dists = nvs.map(i => state.verts[i].cost);
      // Ordering[dists+eucDist, 1] — argmin
      let bestK = 0;
      for (let k = 1; k < nvs.length; k++) {
        if (dists[k] + eucDist[k] < dists[bestK] + eucDist[bestK]) bestK = k;
      }
      if (nvs.length > 0) {
        parent = nvs[bestK];
        nvPos = state.verts[parent].pos;
      }
    }

    success = !inCollision(nvPos, np, obsedges);
  }

  // append new vertex
  const parentVert = state.verts[parent];
  const newVert = {
    pos: np,
    parent: parent + 1,           // 1-based
    index: state.verts.length + 1,
    cost: parentVert.cost + dist(nvPos, np),
    children: [],
  };
  state.verts.push(newVert);
  parentVert.children.push(newVert.index);

  if (isRRTStar) {
    for (let k = 0; k < nvs.length; k++) {
      const nn = nvs[k];
      const newdist = newVert.cost + eucDist[k];
      if (state.verts[nn].cost > newdist) {
        // rewire nn to the new vertex
        const oldParentIdx = state.verts[nn].parent - 1;
        const cp = state.verts[oldParentIdx].children;
        const at = cp.indexOf(nn + 1);
        if (at >= 0) cp.splice(at, 1);
        state.verts[nn].parent = newVert.index;
        state.verts[nn].cost = newdist;
        newVert.children.push(nn + 1);
        updateCostToGo(nn);
      }
    }
  }

  state.newrand = p;
}

// recursive cost refresh after rewiring
function updateCostToGo(index) {
  const v = state.verts[index];
  const parentIdx = v.parent - 1;
  if (parentIdx >= 0) {
    const pv = state.verts[parentIdx];
    v.cost = pv.cost + dist(pv.pos, v.pos);
    for (const c of v.children) updateCostToGo(c - 1);
  }
}

// goal check — port of the reachGoal block
function checkGoal() {
  if (state.reachGoal) return;
  const inRadius = [];
  for (let i = 0; i < state.verts.length; i++) {
    if (dist(state.verts[i].pos, state.goal) <= state.goalRadius) inRadius.push(i);
  }
  if (inRadius.length > 0) {
    let cost = 10000, waypoint = null;
    for (const i of inRadius) {
      if (state.verts[i].cost < cost) {
        waypoint = { pos: state.verts[i].pos.slice(), parent: state.verts[i].parent };
        cost = state.verts[i].cost;
      }
    }
    state.path = [waypoint];
    while (state.path[state.path.length - 1].parent !== 0) {
      const w = state.path[state.path.length - 1];
      state.path.push({
        pos: state.verts[w.parent - 1].pos.slice(),
        parent: state.verts[w.parent - 1].parent,
      });
    }
    state.pathCost = cost;
    state.reachGoal = true;
  }
}

// ---------- rendering ----------

const canvas = document.getElementById("board");
const ctx = canvas.getContext("2d");
const tooltip = document.createElement("div");
tooltip.id = "tooltip";
document.getElementById("stage").appendChild(tooltip);

// PlotRange {{-21,21},{-21,23}} mapped onto the canvas
const XMIN = -21, XMAX = 21, YMIN = -21, YMAX = 23;
const SX = canvas.width / (XMAX - XMIN);
const SY = canvas.height / (YMAX - YMIN);
const toPx = (x, y) => [(x - XMIN) * SX, (YMAX - y) * SY];

function line(a, b) {
  ctx.beginPath();
  ctx.moveTo(...toPx(a[0], a[1]));
  ctx.lineTo(...toPx(b[0], b[1]));
  ctx.stroke();
}

function draw() {
  const obs = OBSTACLES[state.obstacleType];
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.setLineDash([]);

  // white field
  ctx.fillStyle = "#ffffff";
  const [wx0, wy0] = toPx(-20, 20), [wx1, wy1] = toPx(20, -20);
  ctx.fillRect(wx0, wy0, wx1 - wx0, wy1 - wy0);

  // goal disk
  const [gx, gy] = toPx(state.goal[0], state.goal[1]);
  ctx.beginPath();
  ctx.arc(gx, gy, state.goalRadius * SX, 0, 2 * Math.PI);
  ctx.fillStyle = state.reachGoal ? "#2e9e2e" : "#f2d43c";
  ctx.fill();

  // blue border (outside the field) + obstacles
  ctx.fillStyle = "#1a5fb4";
  for (const r of [[[-21,-21],[-20,21]], [[20,-21],[21,21]], [[-21,-21],[21,-20]], [[-21,20],[21,21]]]) {
    const [bx0, by0] = toPx(r[0][0], r[0][1]);
    const [bx1, by1] = toPx(r[1][0], r[1][1]);
    ctx.fillRect(bx0, by0, bx1 - bx0, by1 - by0);
  }
  for (const poly of obs) {
    ctx.beginPath();
    poly.forEach((v, i) => { const [px, py] = toPx(v[0], v[1]); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.closePath();
    ctx.fill();
  }

  // tree edges
  ctx.strokeStyle = "#000";
  ctx.lineWidth = 1;
  for (let n = 1; n < state.verts.length; n++) {
    const v = state.verts[n];
    line(v.pos, state.verts[v.parent - 1].pos);
  }

  // last step: dashed sample line + blue point at new node
  if (state.verts.length > 1 && state.newrand) {
    const last = state.verts[state.verts.length - 1];
    ctx.setLineDash([5, 4]);
    line(last.pos, state.newrand);
    ctx.setLineDash([]);
    ctx.fillStyle = "#2255dd";
    dot(last.pos, 5);
  }

  // red nodes
  ctx.fillStyle = "#e33232";
  for (const v of state.verts) dot(v.pos, 3);

  // green sample point
  if (state.newrand) {
    ctx.fillStyle = "#0a0";
    dot(state.newrand, 5);
  }

  // path
  if (state.reachGoal && state.path.length > 1) {
    ctx.strokeStyle = "#1B6E1B";
    ctx.lineWidth = 3.5;
    ctx.beginPath();
    state.path.forEach((w, i) => { const [px, py] = toPx(w.pos[0], w.pos[1]); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
    ctx.stroke();
    ctx.lineWidth = 1;
  }

  // status line (StringForm in the original)
  const statusEl = document.getElementById("status");
  const nCount = state.verts.length;
  const nText = nCount === 1 ? "1 node" : nCount + " nodes";
  if (state.reachGoal) {
    statusEl.textContent = nText + ", path length " + (Math.round(state.pathCost * 100) / 100);
    statusEl.classList.add("reached");
  } else {
    statusEl.textContent = nText + ", goal not yet reached";
    statusEl.classList.remove("reached");
  }
}

function dot(p, r) {
  const [px, py] = toPx(p[0], p[1]);
  ctx.beginPath();
  ctx.arc(px, py, r, 0, 2 * Math.PI);
  ctx.fill();
}

// ---------- animation loop (mirrors the original's burst behaviour) ----------

let running = false;
let autoPlay = false;

function tick() {
  if (state.pending > 0 || autoPlay) {
    const obs = OBSTACLES[state.obstacleType];
    if (state.pending > 0) {
      // first node of the burst, one dynamic update
      addNode(obs);
      state.pending -= 1;
      state.reachGoal = false;
      // While nodestoadd > 5: batch without redraw
      while (state.pending > 5) {
        addNode(obs);
        state.pending -= 1;
      }
    } else {
      addNode(obs);
      state.reachGoal = false;
    }
    checkGoal();
    draw();
  }
  requestAnimationFrame(tick);
}

// ---------- controls ----------

const treeSel = document.getElementById("treeType");
const obsSel = document.getElementById("obstacleType");
const biasSlider = document.getElementById("explorationBias");
const radiusSlider = document.getElementById("goalRadius");
const autoBtn = document.getElementById("toggleAuto");

treeSel.addEventListener("change", () => { state.treeType = treeSel.value; resetTree(); draw(); });
obsSel.addEventListener("change", () => { state.obstacleType = obsSel.value; resetTree(); draw(); });

biasSlider.addEventListener("input", () => {
  state.explorationBias = parseFloat(biasSlider.value);
  document.getElementById("biasOut").textContent = state.explorationBias;
});
radiusSlider.addEventListener("input", () => {
  state.goalRadius = parseFloat(radiusSlider.value);
  state.reachGoal = false;   // goalRadiusOld change clears the path in the original
  state.path = [];
  document.getElementById("radiusOut").textContent = state.goalRadius;
});

document.querySelectorAll("[data-add]").forEach(btn => {
  btn.addEventListener("click", () => { state.pending += parseInt(btn.dataset.add, 10); });
});

document.getElementById("resetBtn").addEventListener("click", () => { resetTree(); draw(); });

function stopAuto() {
  autoPlay = false;
  autoBtn.classList.remove("active");
  autoBtn.textContent = "auto-play";
}
autoBtn.addEventListener("click", () => {
  autoPlay = !autoPlay;
  autoBtn.classList.toggle("active", autoPlay);
  autoBtn.textContent = autoPlay ? "stop" : "auto-play";
});

// ---------- draggable goal locator ----------

let dragging = false;
function eventWorld(e) {
  const r = canvas.getBoundingClientRect();
  const x = (e.clientX - r.left) / (r.width / (XMAX - XMIN)) + XMIN;
  const y = YMAX - (e.clientY - r.top) / (r.height / (YMAX - YMIN));
  return [x, y];
}
canvas.addEventListener("pointerdown", e => {
  const w = eventWorld(e);
  if (dist(w, state.goal) <= Math.max(state.goalRadius, 1.2)) {
    dragging = true;
    canvas.classList.add("dragging");
    canvas.setPointerCapture(e.pointerId);
  }
});
canvas.addEventListener("pointermove", e => {
  const w = eventWorld(e);
  if (dragging) {
    state.goal = [Math.max(-19, Math.min(19, w[0])), Math.max(-19, Math.min(19, w[1]))];
    state.reachGoal = false;
    state.path = [];
    draw();
  } else {
    // tooltip: nearest red node within a small radius
    let best = null, bestD = 1.0;
    for (const v of state.verts) {
      const d = dist(w, v.pos);
      if (d < bestD) { bestD = d; best = v; }
    }
    if (best) {
      const r = canvas.getBoundingClientRect();
      const [px, py] = toPx(best.pos[0], best.pos[1]);
      tooltip.style.left = (px * r.width / canvas.width) + "px";
      tooltip.style.top = (py * r.height / canvas.height) + "px";
      tooltip.textContent = Math.round(best.cost * 100) / 100;
      tooltip.style.display = "block";
    } else {
      tooltip.style.display = "none";
    }
  }
});
canvas.addEventListener("pointerup", () => { dragging = false; canvas.classList.remove("dragging"); });
canvas.addEventListener("pointerleave", () => { tooltip.style.display = "none"; });

// ---------- go ----------
draw();
requestAnimationFrame(tick);

})();
