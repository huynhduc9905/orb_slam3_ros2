import { describe, expect, it } from "vitest";

import {
  MAP_COLORS,
  occupancyToRGBA,
  isAllUnknown,
  mapWorldBounds,
} from "../src/render/occupancy";
import {
  pathToVertices,
  loopEdgesToSegments,
  PATH_COLORS,
} from "../src/render/paths";
import { scanToPoints, SCAN_COLOR } from "../src/render/scan";
import type { OccupancyGrid, Pose2, Edge2, Point2 } from "../src/types";

function hexToRgba(hex: string): [number, number, number, number] {
  const h = hex.replace("#", "");
  return [
    parseInt(h.slice(0, 2), 16),
    parseInt(h.slice(2, 4), 16),
    parseInt(h.slice(4, 6), 16),
    255,
  ];
}

function makeGrid(
  width: number,
  height: number,
  data: number[],
  resolution = 0.05,
  originX = 0,
  originY = 0,
): OccupancyGrid {
  return {
    header: { stamp: { sec: 0 }, frame_id: "orb_map" },
    info: {
      resolution,
      width,
      height,
      origin: {
        position: { x: originX, y: originY, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    },
    data: Int8Array.from(data),
  };
}

function pixelAt(
  rgba: Uint8ClampedArray,
  width: number,
  x: number,
  y: number,
): [number, number, number, number] {
  const i = (y * width + x) * 4;
  return [rgba[i]!, rgba[i + 1]!, rgba[i + 2]!, rgba[i + 3]!];
}

describe("MAP_COLORS exact palette", () => {
  it("matches the approved layer colors", () => {
    expect(MAP_COLORS.unknown).toBe("#171c21");
    expect(MAP_COLORS.free).toBe("#e5e7eb");
    expect(MAP_COLORS.occupied).toBe("#111827");
    expect(PATH_COLORS.orb).toBe("#38bdf8");
    expect(PATH_COLORS.wheel).toBe("#f59e0b");
    expect(PATH_COLORS.loopEdge).toBe("#e879f9");
    expect(PATH_COLORS.robotOk).toBe("#22c55e");
    expect(SCAN_COLOR).toBe("#facc15");
  });
});

describe("occupancyToRGBA", () => {
  it("produces texture dimensions matching grid width × height", () => {
    const grid = makeGrid(4, 3, [
      0, 0, 0, 0, // row 0
      100, 100, 100, 100, // row 1
      -1, -1, -1, -1, // row 2
    ]);
    const { width, height, data } = occupancyToRGBA(grid);
    expect(width).toBe(4);
    expect(height).toBe(3);
    expect(data).toBeInstanceOf(Uint8ClampedArray);
    expect(data.byteLength).toBe(4 * 3 * 4);
  });

  it("maps free / occupied / unknown to distinct exact pixels", () => {
    // 2×2: free, occupied, unknown, free
    const grid = makeGrid(2, 2, [0, 100, -1, 0]);
    const { width, data } = occupancyToRGBA(grid);

    const free = hexToRgba(MAP_COLORS.free);
    const occupied = hexToRgba(MAP_COLORS.occupied);
    const unknown = hexToRgba(MAP_COLORS.unknown);

    expect(pixelAt(data, width, 0, 0)).toEqual(free);
    expect(pixelAt(data, width, 1, 0)).toEqual(occupied);
    expect(pixelAt(data, width, 0, 1)).toEqual(unknown);
    expect(pixelAt(data, width, 1, 1)).toEqual(free);

    // Distinct from each other
    expect(free).not.toEqual(occupied);
    expect(free).not.toEqual(unknown);
    expect(occupied).not.toEqual(unknown);
  });

  it("treats mid-range occupancy as occupied when >= 50", () => {
    const grid = makeGrid(2, 1, [50, 49]);
    const { width, data } = occupancyToRGBA(grid);
    expect(pixelAt(data, width, 0, 0)).toEqual(hexToRgba(MAP_COLORS.occupied));
    expect(pixelAt(data, width, 1, 0)).toEqual(hexToRgba(MAP_COLORS.free));
  });

  it("handles zero-sized map without throwing", () => {
    const grid = makeGrid(0, 0, []);
    const { width, height, data } = occupancyToRGBA(grid);
    expect(width).toBe(0);
    expect(height).toBe(0);
    expect(data.byteLength).toBe(0);
  });
});

describe("isAllUnknown / mapWorldBounds", () => {
  it("detects all-unknown and zero-sized maps", () => {
    expect(isAllUnknown(makeGrid(0, 0, []))).toBe(true);
    expect(isAllUnknown(makeGrid(2, 2, [-1, -1, -1, -1]))).toBe(true);
    expect(isAllUnknown(makeGrid(2, 1, [-1, 0]))).toBe(false);
  });

  it("computes world bounds from origin, size, and resolution", () => {
    const grid = makeGrid(10, 20, new Array(200).fill(0), 0.1, -1, -2);
    const b = mapWorldBounds(grid);
    expect(b.minX).toBeCloseTo(-1);
    expect(b.minY).toBeCloseTo(-2);
    expect(b.maxX).toBeCloseTo(-1 + 10 * 0.1);
    expect(b.maxY).toBeCloseTo(-2 + 20 * 0.1);
    expect(b.width).toBeCloseTo(1);
    expect(b.height).toBeCloseTo(2);
  });
});

describe("pathToVertices", () => {
  it("flattens poses into x,y vertex pairs for ORB / wheel paths", () => {
    const poses: Pose2[] = [
      { x: 0, y: 0, yaw: 0 },
      { x: 1.5, y: 2.5, yaw: 0.1 },
      { x: 3, y: 4, yaw: 0 },
    ];
    const verts = pathToVertices(poses);
    expect(verts).toEqual(new Float32Array([0, 0, 1.5, 2.5, 3, 4]));
  });

  it("returns empty vertices for empty path", () => {
    expect(pathToVertices([])).toEqual(new Float32Array([]));
  });
});

describe("loopEdgesToSegments", () => {
  it("emits from/to pairs as distinct segment vertices", () => {
    const edges: Edge2[] = [
      {
        from: { x: 0, y: 0, yaw: 0 },
        to: { x: 1, y: 1, yaw: 0 },
      },
      {
        from: { x: 2, y: 0, yaw: 0 },
        to: { x: 2, y: 3, yaw: 0 },
      },
    ];
    const segs = loopEdgesToSegments(edges);
    // each edge → 4 floats (x0,y0,x1,y1)
    expect(segs).toEqual(new Float32Array([0, 0, 1, 1, 2, 0, 2, 3]));
  });
});

describe("scanToPoints", () => {
  it("flattens provisional scan points for distinct yellow layer", () => {
    const points: Point2[] = [
      { x: 0.1, y: 0.2 },
      { x: 0.3, y: 0.4 },
    ];
    expect(scanToPoints(points)).toEqual(new Float32Array([0.1, 0.2, 0.3, 0.4]));
  });
});

describe("layer pixel distinctness (composited sample)", () => {
  it("ORB path, wheel path, scan, and loop-edge colors are all distinct", () => {
    const colors = [
      PATH_COLORS.orb,
      PATH_COLORS.wheel,
      SCAN_COLOR,
      PATH_COLORS.loopEdge,
      MAP_COLORS.free,
      MAP_COLORS.occupied,
      MAP_COLORS.unknown,
      PATH_COLORS.robotOk,
    ];
    const unique = new Set(colors);
    expect(unique.size).toBe(colors.length);
  });
});
