/**
 * Read-only Foxglove WebSocket client for the ORB lidar dashboard.
 *
 * Intentionally does NOT import or call advertise / sendMessage /
 * sendServiceCallRequest / getParameters / setParameters / asset /
 * connection-graph APIs from @foxglove/ws-protocol.
 */

import { parse } from "@foxglove/rosmsg";
import { MessageReader } from "@foxglove/rosmsg2-serialization";
import { FoxgloveClient } from "@foxglove/ws-protocol";
import type { Channel, IWebSocket, SubscriptionId } from "@foxglove/ws-protocol";

import type { DashboardStore } from "./store";
import type {
  DashboardEvent,
  DashboardEventType,
  Edge2,
  HealthSummary,
  OccupancyGrid,
  Point2,
  Pose2,
  TrackingStateName,
  TrackingSummary,
} from "./types";

/** Exact topic whitelist from the dashboard plan. */
export const ALLOWED_TOPICS: ReadonlySet<string> = new Set([
  "/orb_lidar/map",
  "/orb_lidar/map_revision",
  "/orb_lidar/corrected_path_revisioned",
  "/orb_lidar/wheel_path",
  "/orb_lidar/provisional_scan",
  "/orb_slam3/tracked_frame",
  "/orb_slam3/events",
  "/orb_slam3/keyframes",
  "/orb_slam3/loop_edges",
  "/orb_slam3/tracking_image/compressed",
  "/diagnostics",
]);

export type WebSocketFactory = (
  url: string,
  protocols?: string | string[],
) => WebSocket;

export interface DashboardConnectionOptions {
  url: string;
  store: DashboardStore;
  /** Injectable for tests; defaults to global WebSocket. */
  webSocketFactory?: WebSocketFactory;
}

export interface DashboardConnection {
  connect(): void;
  close(): void;
}

interface ReaderEntry {
  topic: string;
  reader: MessageReader;
}

// ── Geometry helpers ────────────────────────────────────────────────────────

function asBigInt(value: unknown): bigint {
  if (typeof value === "bigint") return value;
  if (typeof value === "number") return BigInt(Math.trunc(value));
  if (typeof value === "string") return BigInt(value);
  return 0n;
}

function asNumber(value: unknown, fallback = 0): number {
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}

function yawFromQuaternion(q: {
  x?: number;
  y?: number;
  z?: number;
  w?: number;
}): number {
  const x = q.x ?? 0;
  const y = q.y ?? 0;
  const z = q.z ?? 0;
  const w = q.w ?? 1;
  // yaw (Z) from quaternion — planar map assumption
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function pose2FromGeometry(pose: {
  position?: { x?: number; y?: number; z?: number };
  orientation?: { x?: number; y?: number; z?: number; w?: number };
}): Pose2 {
  return {
    x: asNumber(pose.position?.x),
    y: asNumber(pose.position?.y),
    yaw: yawFromQuaternion(pose.orientation ?? {}),
  };
}

function pathFromPoseStampedArray(poses: unknown): Pose2[] {
  if (!Array.isArray(poses)) return [];
  return poses.map((p) =>
    pose2FromGeometry(
      (p as { pose?: Parameters<typeof pose2FromGeometry>[0] }).pose ?? {},
    ),
  );
}

const TRACKING_STATE_NAMES: TrackingStateName[] = [
  "NO_IMAGES_YET",
  "NOT_INITIALIZED",
  "OK",
  "RECENTLY_LOST",
  "LOST",
];

const EVENT_TYPE_NAMES: DashboardEventType[] = [
  "INITIALIZED",
  "LOST",
  "RELOCALIZED",
  "LOOP_CLOSED",
  "MAP_CREATED",
  "MAP_MERGED",
  "MAP_RESET",
];

// ── Topic handlers ──────────────────────────────────────────────────────────

function handleMessage(
  topic: string,
  decoded: Record<string, unknown>,
  store: DashboardStore,
): void {
  switch (topic) {
    case "/orb_lidar/map": {
      const info = decoded.info as {
        resolution?: number;
        width?: number;
        height?: number;
        origin?: OccupancyGrid["info"]["origin"];
      };
      const dataRaw = decoded.data;
      let data: Int8Array | number[];
      if (dataRaw instanceof Int8Array) {
        data = dataRaw;
      } else if (Array.isArray(dataRaw)) {
        data = Int8Array.from(dataRaw as number[]);
      } else if (dataRaw instanceof Uint8Array) {
        data = new Int8Array(dataRaw.buffer, dataRaw.byteOffset, dataRaw.byteLength);
      } else {
        data = [];
      }
      const header = decoded.header as OccupancyGrid["header"];
      const map: OccupancyGrid = {
        header: header ?? { stamp: { sec: 0 }, frame_id: "" },
        info: {
          resolution: asNumber(info?.resolution, 0.05),
          width: asNumber(info?.width),
          height: asNumber(info?.height),
          origin: info?.origin ?? {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
        data,
      };
      store.setMap(map);
      break;
    }

    case "/orb_lidar/map_revision": {
      store.setMapRevision(
        asBigInt(decoded.graph_revision),
        asBigInt(decoded.map_revision),
      );
      break;
    }

    case "/orb_lidar/corrected_path_revisioned": {
      const revision = asBigInt(decoded.graph_revision);
      const path = pathFromPoseStampedArray(decoded.poses);
      store.setOrbPath(revision, path);
      break;
    }

    case "/orb_lidar/wheel_path": {
      store.setWheelPath(pathFromPoseStampedArray(decoded.poses));
      break;
    }

    case "/orb_lidar/provisional_scan": {
      // visualization_msgs/Marker — POINTS type uses points[]
      const pointsRaw = decoded.points;
      const points: Point2[] = Array.isArray(pointsRaw)
        ? pointsRaw.map((p: { x?: number; y?: number }) => ({
            x: asNumber(p.x),
            y: asNumber(p.y),
          }))
        : [];
      store.setProvisionalScan(points);
      break;
    }

    case "/orb_slam3/tracked_frame": {
      const stateIdx = asNumber(decoded.tracking_state);
      const tracking: TrackingSummary = {
        state: TRACKING_STATE_NAMES[stateIdx] ?? "UNKNOWN",
        poseValid: Boolean(decoded.pose_valid),
        trackedKeypoints: asNumber(decoded.tracked_keypoints),
        pose: pose2FromGeometry(
          (decoded.pose as Parameters<typeof pose2FromGeometry>[0]) ?? {},
        ),
        mapId: asBigInt(decoded.map_id),
        referenceKeyframeId: asBigInt(decoded.reference_keyframe_id),
      };
      store.setTracking(tracking, asBigInt(decoded.graph_revision));
      break;
    }

    case "/orb_slam3/events": {
      const typeIdx = asNumber(decoded.type);
      const header = decoded.header as {
        stamp?: { sec?: number; nsec?: number; nanosec?: number };
      };
      const stamp = header?.stamp ?? {};
      const event: DashboardEvent = {
        type: EVENT_TYPE_NAMES[typeIdx] ?? "UNKNOWN",
        graphRevision: asBigInt(decoded.graph_revision),
        mapId: asBigInt(decoded.map_id),
        keyframeId: asBigInt(decoded.keyframe_id),
        detail: typeof decoded.detail === "string" ? decoded.detail : "",
        stampSec: asNumber(stamp.sec),
        stampNsec: asNumber(stamp.nsec ?? stamp.nanosec),
      };
      store.appendEvent(event);
      break;
    }

    case "/orb_slam3/keyframes": {
      // visualization_msgs/MarkerArray — markers[].pose
      // Always update, including empty arrays, so stale keyframes clear.
      const markers = (decoded.markers as unknown[]) ?? [];
      if (Array.isArray(markers)) {
        const poses: Pose2[] = markers.map((m) =>
          pose2FromGeometry(
            (m as { pose?: Parameters<typeof pose2FromGeometry>[0] }).pose ?? {},
          ),
        );
        store.setKeyframes(poses);
      } else if (Array.isArray(decoded.poses)) {
        // Fallback if a Path-like schema is advertised
        store.setKeyframes(pathFromPoseStampedArray(decoded.poses));
      }
      break;
    }

    case "/orb_slam3/loop_edges": {
      // MarkerArray LINE_LIST: points come in pairs
      const markers = (decoded.markers as unknown[]) ?? [];
      const edges: Edge2[] = [];
      if (Array.isArray(markers)) {
        for (const m of markers) {
          const pts = (m as { points?: Array<{ x?: number; y?: number }> }).points;
          if (!Array.isArray(pts)) continue;
          for (let i = 0; i + 1 < pts.length; i += 2) {
            edges.push({
              from: { x: asNumber(pts[i]?.x), y: asNumber(pts[i]?.y), yaw: 0 },
              to: {
                x: asNumber(pts[i + 1]?.x),
                y: asNumber(pts[i + 1]?.y),
                yaw: 0,
              },
            });
          }
        }
      }
      store.setLoopEdges(edges);
      break;
    }

    case "/orb_slam3/tracking_image/compressed": {
      const format =
        typeof decoded.format === "string" ? decoded.format : "jpeg";
      const dataRaw = decoded.data;
      let bytes: Uint8Array;
      if (dataRaw instanceof Uint8Array) {
        bytes = dataRaw;
      } else if (Array.isArray(dataRaw)) {
        bytes = Uint8Array.from(dataRaw as number[]);
      } else {
        bytes = new Uint8Array();
      }
      const mime =
        format.includes("png")
          ? "image/png"
          : format.includes("jpeg") || format.includes("jpg")
            ? "image/jpeg"
            : "application/octet-stream";
      if (typeof Blob !== "undefined" && typeof URL !== "undefined") {
        // Copy into a fresh ArrayBuffer-backed view for BlobPart typing (TS 7).
        const copy = new Uint8Array(bytes.byteLength);
        copy.set(bytes);
        const blob = new Blob([copy], { type: mime });
        const url = URL.createObjectURL(blob);
        store.setTrackingImageUrl(url);
      }
      break;
    }

    case "/diagnostics": {
      const statusArr = (decoded.status as unknown[]) ?? [];
      const items: HealthSummary["items"] = [];
      let worst: HealthSummary["level"] = "ok";
      let message = "";
      if (Array.isArray(statusArr)) {
        for (const s of statusArr) {
          const st = s as {
            name?: string;
            level?: number | string;
            message?: string;
          };
          const levelNum = asNumber(st.level);
          const levelName =
            levelNum >= 2 ? "error" : levelNum === 1 ? "warn" : "ok";
          if (levelName === "error") worst = "error";
          else if (levelName === "warn" && worst === "ok") worst = "warn";
          items.push({
            name: st.name ?? "",
            level: levelName,
            message: st.message ?? "",
          });
          if (!message && st.message) message = st.message;
        }
      }
      store.setHealth({ level: worst, message, items });
      break;
    }

    default:
      // Whitelist already filtered; ignore unknown.
      break;
  }
}

// ── Connection ──────────────────────────────────────────────────────────────

export function createDashboardConnection(
  options: DashboardConnectionOptions,
): DashboardConnection {
  const { url, store } = options;
  const webSocketFactory =
    options.webSocketFactory ??
    ((u, p) => new WebSocket(u, p as string | string[] | undefined));

  let ws: WebSocket | undefined;
  let client: FoxgloveClient | undefined;
  let closed = false;
  const readers = new Map<SubscriptionId, ReaderEntry>();

  function attachClient(socket: WebSocket): void {
    // FoxgloveClient only needs IWebSocket surface. Cast bridges DOM WebSocket
    // vs Node-style IWebSocket send signature differences under TS 7.
    client = new FoxgloveClient({ ws: socket as unknown as IWebSocket });

    client.on("open", () => {
      if (!closed) store.setConnection("connected");
    });

    client.on("close", () => {
      // Preserve last display state; only flip connection flag.
      if (!closed) store.setConnection("disconnected");
    });

    client.on("error", () => {
      if (!closed) store.setConnection("error");
    });

    client.on("advertise", (channels: Channel[]) => {
      if (!client) return;
      for (const channel of channels) {
        if (!ALLOWED_TOPICS.has(channel.topic) || channel.encoding !== "cdr") {
          continue;
        }
        try {
          const definitions = parse(channel.schema, { ros2: true });
          const reader = new MessageReader(definitions, {
            timeType: "sec,nsec",
          });
          const subscriptionId = client.subscribe(channel.id);
          readers.set(subscriptionId, { topic: channel.topic, reader });
        } catch {
          // Schema parse failure: skip channel; keep connection alive.
        }
      }
    });

    client.on("message", (event) => {
      const entry = readers.get(event.subscriptionId);
      if (!entry) return;
      try {
        const decoded = entry.reader.readMessage<Record<string, unknown>>(
          event.data,
        );
        handleMessage(entry.topic, decoded, store);
      } catch {
        // Drop corrupt frames; do not tear down the session.
      }
    });
  }

  return {
    connect(): void {
      closed = false;
      store.setConnection("connecting");
      readers.clear();
      ws = webSocketFactory(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]);
      attachClient(ws);
    },

    close(): void {
      closed = true;
      try {
        client?.close();
      } catch {
        // ignore
      }
      try {
        ws?.close();
      } catch {
        // ignore
      }
      client = undefined;
      ws = undefined;
      readers.clear();
      // Do not wipe display state — leave connection as-is or disconnected.
      if (store.getState().connection !== "disconnected") {
        store.setConnection("disconnected");
      }
    },
  };
}
