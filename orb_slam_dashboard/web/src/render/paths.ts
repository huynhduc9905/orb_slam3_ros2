import type { Edge2, Pose2 } from "../types";

/** Exact path / edge / robot colors from the dashboard plan. */
export const PATH_COLORS = {
  orb: "#38bdf8",
  wheel: "#f59e0b",
  loopEdge: "#e879f9",
  robotOk: "#22c55e",
} as const;

/** Flatten Pose2[] into interleaved x,y vertices for a polyline layer. */
export function pathToVertices(path: Pose2[]): Float32Array {
  const out = new Float32Array(path.length * 2);
  for (let i = 0; i < path.length; i++) {
    out[i * 2] = path[i]!.x;
    out[i * 2 + 1] = path[i]!.y;
  }
  return out;
}

/** Flatten Edge2[] into segment pairs (x0,y0,x1,y1) for LINE_LIST style drawing. */
export function loopEdgesToSegments(edges: Edge2[]): Float32Array {
  const out = new Float32Array(edges.length * 4);
  for (let i = 0; i < edges.length; i++) {
    const e = edges[i]!;
    const o = i * 4;
    out[o] = e.from.x;
    out[o + 1] = e.from.y;
    out[o + 2] = e.to.x;
    out[o + 3] = e.to.y;
  }
  return out;
}
