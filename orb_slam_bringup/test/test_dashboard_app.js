"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");
const vm = require("node:vm");

function loadPanelBounds() {
  const drawingContext = {
    beginPath() {}, arc() {}, fill() {}, fillRect() {}, fillText() {}, lineTo() {},
    moveTo() {}, setLineDash() {}, stroke() {},
  };
  const element = () => ({
    addEventListener() {}, appendChild() {}, classList: { toggle() {} },
    getBoundingClientRect: () => ({ width: 300, height: 200 }), getContext: () => drawingContext,
    hidden: false, setAttribute() {}, style: {}, textContent: "", width: 300, height: 200,
  });
  const elements = new Map();
  const context = {
    Image: class {},
    document: { createElement: element, getElementById: (id) => {
      if (!elements.has(id)) elements.set(id, element());
      return elements.get(id);
    } },
    fetch: async () => { throw new Error("not connected"); },
    performance: { now: () => 0 },
    setInterval: () => 0,
    setTimeout: () => 0,
    window: { addEventListener() {}, devicePixelRatio: 1 },
  };
  context.globalThis = context;
  vm.createContext(context);
  const app = fs.readFileSync(path.join(__dirname, "..", "web", "app.js"), "utf8");
  vm.runInContext(`${app}\nglobalThis.panelBoundsForTest = panelBounds;`, context);
  return context.panelBoundsForTest;
}

test("outlying displayed loop edge contributes to graph viewport bounds", () => {
  const panelBounds = loadPanelBounds();
  const edge = { from: { x: -1000, y: 400 }, to: { x: 1200, y: -600 } };
  const bounds = panelBounds([], [{ id: 0, x: 0, y: 0 }], null, [edge]);

  assert.equal(bounds.minX, edge.from.x);
  assert.equal(bounds.maxX, edge.to.x);
  assert.equal(bounds.minY, edge.to.y);
  assert.equal(bounds.maxY, edge.from.y);

  // `drawPanel` adds 20% padding (half-range * 0.6), so both endpoints lie
  // within the 300x200 graph canvas even though neither is a displayed node.
  const cx = (bounds.minX + bounds.maxX) / 2;
  const cy = (bounds.minY + bounds.maxY) / 2;
  const half = Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY, 0.5) * 0.6;
  const scale = 200 / (2 * half);
  for (const point of [edge.from, edge.to]) {
    const sx = 150 + (point.x - cx) * scale;
    const sy = 100 - (point.y - cy) * scale;
    assert.ok(sx >= 0 && sx <= 300);
    assert.ok(sy >= 0 && sy <= 200);
  }
});
