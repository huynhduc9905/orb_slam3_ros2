/** Connection lifecycle for the read-only Foxglove client. */
export type ConnectionState =
  | "connecting"
  | "connected"
  | "disconnected"
  | "error";

/** Planar pose used by map/path overlays (map frame). */
export interface Pose2 {
  x: number;
  y: number;
  yaw: number;
}

/** Planar point (scan markers). */
export interface Point2 {
  x: number;
  y: number;
}

/** Undirected loop-closure edge in map frame. */
export interface Edge2 {
  from: Pose2;
  to: Pose2;
}

/** Subset of nav_msgs/OccupancyGrid needed by the map canvas. */
export interface OccupancyGridInfo {
  resolution: number;
  width: number;
  height: number;
  origin: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
}

export interface OccupancyGrid {
  header: {
    stamp: { sec: number; nsec?: number; nanosec?: number };
    frame_id: string;
  };
  info: OccupancyGridInfo;
  /** Occupancy values: -1 unknown, 0 free, 100 occupied (ROS convention). */
  data: Int8Array | number[];
}

/** Tracking state names matching orb_slam3_msgs/TrackedFrame constants. */
export type TrackingStateName =
  | "NO_IMAGES_YET"
  | "NOT_INITIALIZED"
  | "OK"
  | "RECENTLY_LOST"
  | "LOST"
  | "UNKNOWN";

export interface TrackingSummary {
  state: TrackingStateName;
  poseValid: boolean;
  trackedKeypoints: number;
  pose?: Pose2;
  mapId: bigint;
  referenceKeyframeId: bigint;
}

export interface HealthSummary {
  level: "ok" | "warn" | "error" | "stale" | "unknown";
  message: string;
  /** Optional per-status lines from diagnostic_msgs. */
  items: Array<{ name: string; level: string; message: string }>;
}

export type DashboardEventType =
  | "INITIALIZED"
  | "LOST"
  | "RELOCALIZED"
  | "LOOP_CLOSED"
  | "MAP_CREATED"
  | "MAP_MERGED"
  | "MAP_RESET"
  | "UNKNOWN";

export interface DashboardEvent {
  type: DashboardEventType;
  graphRevision: bigint;
  mapId: bigint;
  keyframeId: bigint;
  detail: string;
  stampSec: number;
  stampNsec: number;
}

export interface DashboardState {
  connection: ConnectionState;
  map?: OccupancyGrid;
  graphRevision: bigint;
  mapRevision: bigint;
  orbPath: Pose2[];
  wheelPath: Pose2[];
  provisionalScan: Point2[];
  keyframes: Pose2[];
  loopEdges: Edge2[];
  tracking: TrackingSummary;
  health: HealthSummary;
  events: DashboardEvent[];
  trackingImageUrl?: string;
}

export const MAX_EVENT_HISTORY = 2000;

export function createInitialDashboardState(): DashboardState {
  return {
    connection: "disconnected",
    graphRevision: 0n,
    mapRevision: 0n,
    orbPath: [],
    wheelPath: [],
    provisionalScan: [],
    keyframes: [],
    loopEdges: [],
    tracking: {
      state: "NO_IMAGES_YET",
      poseValid: false,
      trackedKeypoints: 0,
      mapId: 0n,
      referenceKeyframeId: 0n,
    },
    health: {
      level: "unknown",
      message: "",
      items: [],
    },
    events: [],
  };
}
