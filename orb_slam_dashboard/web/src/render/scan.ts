import type { Point2 } from "../types";

/** Provisional scan marker color (exact). */
export const SCAN_COLOR = "#facc15";

/** Flatten Point2[] into interleaved x,y for a points layer. */
export function scanToPoints(points: Point2[]): Float32Array {
  const out = new Float32Array(points.length * 2);
  for (let i = 0; i < points.length; i++) {
    out[i * 2] = points[i]!.x;
    out[i * 2 + 1] = points[i]!.y;
  }
  return out;
}
