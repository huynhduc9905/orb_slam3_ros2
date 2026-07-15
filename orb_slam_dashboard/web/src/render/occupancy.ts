import type { OccupancyGrid } from "../types";

/** Exact map layer colors from the dashboard plan. */
export const MAP_COLORS = {
  unknown: "#171c21",
  free: "#e5e7eb",
  occupied: "#111827",
} as const;

function hexToRgb(hex: string): [number, number, number] {
  const h = hex.replace("#", "");
  return [
    parseInt(h.slice(0, 2), 16),
    parseInt(h.slice(2, 4), 16),
    parseInt(h.slice(4, 6), 16),
  ];
}

const RGB_UNKNOWN = hexToRgb(MAP_COLORS.unknown);
const RGB_FREE = hexToRgb(MAP_COLORS.free);
const RGB_OCCUPIED = hexToRgb(MAP_COLORS.occupied);

export interface OccupancyRgba {
  width: number;
  height: number;
  data: Uint8ClampedArray;
}

/**
 * Convert OccupancyGrid.data to one RGBA texture buffer.
 * ROS convention: -1 unknown, 0 free, 100 occupied; mid values >= 50 → occupied.
 * Row-major, y-up in ROS map frame → stored with row 0 at origin (bottom in ROS).
 * Pixi/canvas y-down: callers may flip; we keep ROS row order (row 0 = y=0).
 */
export function occupancyToRGBA(grid: OccupancyGrid): OccupancyRgba {
  const width = grid.info.width | 0;
  const height = grid.info.height | 0;
  if (width <= 0 || height <= 0) {
    return { width: Math.max(0, width), height: Math.max(0, height), data: new Uint8ClampedArray(0) };
  }
  const src = grid.data;
  const data = new Uint8ClampedArray(width * height * 4);
  const len = width * height;
  for (let i = 0; i < len; i++) {
    const v = typeof src[i] === "number" ? (src[i] as number) : 0;
    let r: number;
    let g: number;
    let b: number;
    if (v < 0) {
      [r, g, b] = RGB_UNKNOWN;
    } else if (v >= 50) {
      [r, g, b] = RGB_OCCUPIED;
    } else {
      [r, g, b] = RGB_FREE;
    }
    const o = i * 4;
    data[o] = r;
    data[o + 1] = g;
    data[o + 2] = b;
    data[o + 3] = 255;
  }
  return { width, height, data };
}

/** True when map is missing cells, zero-sized, or every cell is unknown (-1). */
export function isAllUnknown(grid: OccupancyGrid | undefined): boolean {
  if (!grid) return true;
  const w = grid.info.width | 0;
  const h = grid.info.height | 0;
  if (w <= 0 || h <= 0) return true;
  const src = grid.data;
  const len = w * h;
  if (src.length < len) return true;
  for (let i = 0; i < len; i++) {
    const v = typeof src[i] === "number" ? (src[i] as number) : -1;
    if (v >= 0) return false;
  }
  return true;
}

export interface WorldBounds {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
  width: number;
  height: number;
}

/** World-frame AABB of the occupancy grid from origin + size * resolution. */
export function mapWorldBounds(grid: OccupancyGrid): WorldBounds {
  const res = grid.info.resolution;
  const w = grid.info.width * res;
  const h = grid.info.height * res;
  const minX = grid.info.origin.position.x;
  const minY = grid.info.origin.position.y;
  return {
    minX,
    minY,
    maxX: minX + w,
    maxY: minY + h,
    width: w,
    height: h,
  };
}
