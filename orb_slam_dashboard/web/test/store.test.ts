import { describe, expect, it, vi, beforeEach, afterEach } from "vitest";
import { parse } from "@foxglove/rosmsg";
import { MessageWriter } from "@foxglove/rosmsg2-serialization";
import type { Channel } from "@foxglove/ws-protocol";

import {
  ALLOWED_TOPICS,
  createDashboardConnection,
  type DashboardConnection,
  type WebSocketFactory,
} from "../src/foxglove";
import {
  createDashboardStore,
  type DashboardStore,
} from "../src/store";
import type { DashboardState } from "../src/types";

// ── ROS 2 message schemas used by fixtures ──────────────────────────────────

const HEADER_DEFS = `
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
`;

const MAP_META_DEFS = `
================================================================================
MSG: nav_msgs/MapMetaData
builtin_interfaces/Time map_load_time
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
`;

const OCCUPANCY_GRID_SCHEMA = `
std_msgs/Header header
nav_msgs/MapMetaData info
int8[] data
${HEADER_DEFS}
${MAP_META_DEFS}
`;

const REVISIONED_PATH_SCHEMA = `
std_msgs/Header header
uint64 graph_revision
geometry_msgs/PoseStamped[] poses
${HEADER_DEFS}
================================================================================
MSG: geometry_msgs/PoseStamped
std_msgs/Header header
geometry_msgs/Pose pose
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
`;

const MAP_REVISION_SCHEMA = `
std_msgs/Header header
uint8 state
uint64 graph_revision
uint64 map_revision
uint64 input_scan_count
uint64 committed_scan_count
float64 duration_ms
string detail
${HEADER_DEFS}
`;

const NAV_PATH_SCHEMA = `
std_msgs/Header header
geometry_msgs/PoseStamped[] poses
${HEADER_DEFS}
================================================================================
MSG: geometry_msgs/PoseStamped
std_msgs/Header header
geometry_msgs/Pose pose
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
`;

const TRACKING_EVENT_SCHEMA = `
std_msgs/Header header
uint8 type
uint64 graph_revision
uint64 map_id
uint64 keyframe_id
string detail
${HEADER_DEFS}
`;

const TRACKED_FRAME_SCHEMA = `
std_msgs/Header header
uint8 tracking_state
bool pose_valid
uint64 map_id
uint64 reference_keyframe_id
geometry_msgs/Pose pose
geometry_msgs/Transform reference_to_frame
uint64 graph_revision
uint32 tracked_keypoints
${HEADER_DEFS}
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Transform
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
`;

function serialize(schema: string, message: unknown): Uint8Array {
  const definitions = parse(schema, { ros2: true });
  const writer = new MessageWriter(definitions);
  return writer.writeMessage(message);
}

function identityPose() {
  return {
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 },
  };
}

function poseAt(x: number, y: number, yaw = 0) {
  const half = yaw / 2;
  return {
    position: { x, y, z: 0 },
    orientation: { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) },
  };
}

function stampedPose(x: number, y: number, yaw = 0) {
  return {
    header: { stamp: { sec: 1, nanosec: 0 }, frame_id: "orb_map" },
    pose: poseAt(x, y, yaw),
  };
}

// ── Fake WebSocket that FoxgloveClient can drive ────────────────────────────

type WsListener = ((event: unknown) => void) | null;

class FakeWebSocket {
  static OPEN = 1;
  static CLOSED = 3;

  binaryType = "arraybuffer";
  protocol = "foxglove.websocket.v1";
  readyState = FakeWebSocket.OPEN;
  sent: string[] = [];
  onerror: WsListener = null;
  onopen: WsListener = null;
  onclose: WsListener = null;
  onmessage: WsListener = null;

  constructor(
    public readonly url: string,
    public readonly protocols?: string | string[],
  ) {
    queueMicrotask(() => this.onopen?.({ type: "open" }));
  }

  send(data: string | ArrayBuffer | ArrayBufferView): void {
    if (typeof data === "string") {
      this.sent.push(data);
    }
  }

  close(): void {
    this.readyState = FakeWebSocket.CLOSED;
    this.onclose?.({ type: "close", code: 1000, reason: "", wasClean: true });
  }

  /** Deliver a JSON server message (advertise, serverInfo, …). */
  deliverJson(payload: unknown): void {
    this.onmessage?.({ data: JSON.stringify(payload) });
  }

  /** Deliver a binary MESSAGE_DATA frame for a subscription. */
  deliverMessageData(
    subscriptionId: number,
    timestampNs: bigint,
    cdr: Uint8Array,
  ): void {
    const buf = new ArrayBuffer(1 + 4 + 8 + cdr.byteLength);
    const view = new DataView(buf);
    view.setUint8(0, 1); // BinaryOpcode.MESSAGE_DATA
    view.setUint32(1, subscriptionId, true);
    view.setBigUint64(5, timestampNs, true);
    new Uint8Array(buf, 13).set(cdr);
    this.onmessage?.({ data: buf });
  }
}

function makeChannel(
  id: number,
  topic: string,
  schemaName: string,
  schema: string,
  encoding = "cdr",
): Channel {
  return {
    id,
    topic,
    encoding,
    schemaName,
    schema,
    schemaEncoding: "ros2msg",
  };
}

function parseSubscribeOps(ws: FakeWebSocket): Array<{ id: number; channelId: number }> {
  const subs: Array<{ id: number; channelId: number }> = [];
  for (const raw of ws.sent) {
    const msg = JSON.parse(raw) as {
      op?: string;
      subscriptions?: Array<{ id: number; channelId: number }>;
    };
    if (msg.op === "subscribe" && msg.subscriptions) {
      subs.push(...msg.subscriptions);
    }
  }
  return subs;
}

function flushRaf(): Promise<void> {
  return new Promise((resolve) => {
    // Drive both rAF (production path) and a microtask fallback.
    if (typeof requestAnimationFrame === "function") {
      requestAnimationFrame(() => resolve());
    } else {
      queueMicrotask(() => resolve());
    }
  });
}

async function waitForState(
  store: DashboardStore,
  predicate: (s: DashboardState) => boolean,
  timeoutMs = 500,
): Promise<DashboardState> {
  const start = Date.now();
  while (Date.now() - start < timeoutMs) {
    const snap = store.getState();
    if (predicate(snap)) return snap;
    await flushRaf();
    await new Promise((r) => setTimeout(r, 5));
  }
  throw new Error(
    `waitForState timed out; last state: ${JSON.stringify(store.getState(), (_, v) =>
      typeof v === "bigint" ? v.toString() : v,
    )}`,
  );
}

describe("ALLOWED_TOPICS whitelist", () => {
  it("contains exactly the eleven approved topics", () => {
    const expected = [
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
    ];
    expect([...ALLOWED_TOPICS].sort()).toEqual([...expected].sort());
    expect(ALLOWED_TOPICS.size).toBe(11);
  });
});

describe("read-only Foxglove client + dashboard store", () => {
  let lastWs: FakeWebSocket | undefined;
  let store: DashboardStore;
  let connection: DashboardConnection;
  let snapshots: DashboardState[];

  const wsFactory: WebSocketFactory = (url, protocols) => {
    lastWs = new FakeWebSocket(url, protocols);
    return lastWs as unknown as WebSocket;
  };

  beforeEach(() => {
    lastWs = undefined;
    snapshots = [];
    store = createDashboardStore();
    store.subscribe((s) => {
      snapshots.push(s);
    });
    connection = createDashboardConnection({
      url: "ws://127.0.0.1:8765",
      store,
      webSocketFactory: wsFactory,
    });
  });

  afterEach(() => {
    connection.close();
  });

  async function openAndAdvertise(channels: Channel[]): Promise<FakeWebSocket> {
    connection.connect();
    await flushRaf();
    const ws = lastWs;
    if (!ws) throw new Error("WebSocket was not created");
    // Wait for open microtask
    await new Promise((r) => setTimeout(r, 0));
    ws.deliverJson({
      op: "serverInfo",
      name: "test",
      capabilities: [],
    });
    ws.deliverJson({ op: "advertise", channels });
    await new Promise((r) => setTimeout(r, 0));
    return ws;
  }

  it("subscribes only to allowed topics with cdr encoding", async () => {
    const channels = [
      makeChannel(1, "/orb_lidar/map", "nav_msgs/msg/OccupancyGrid", OCCUPANCY_GRID_SCHEMA),
      makeChannel(2, "/orb_lidar/map_revision", "orb_slam3_msgs/msg/MapRevision", MAP_REVISION_SCHEMA),
      makeChannel(
        3,
        "/orb_lidar/corrected_path_revisioned",
        "orb_slam3_msgs/msg/RevisionedPath",
        REVISIONED_PATH_SCHEMA,
      ),
      makeChannel(4, "/forbidden/cmd_vel", "geometry_msgs/msg/Twist", "float64 linear\nfloat64 angular"),
      makeChannel(
        5,
        "/orb_lidar/wheel_path",
        "nav_msgs/msg/Path",
        NAV_PATH_SCHEMA,
        "json", // wrong encoding — must be ignored
      ),
      makeChannel(6, "/orb_slam3/events", "orb_slam3_msgs/msg/TrackingEvent", TRACKING_EVENT_SCHEMA),
    ];

    const ws = await openAndAdvertise(channels);
    const subs = parseSubscribeOps(ws);
    const subscribedChannelIds = subs.map((s) => s.channelId).sort();

    expect(subscribedChannelIds).toEqual([1, 2, 3, 6]);
    // Never sent advertise/publish/service/param ops
    for (const raw of ws.sent) {
      const msg = JSON.parse(raw) as { op: string };
      expect(["subscribe"]).toContain(msg.op);
    }
  });

  it("decodes ROS 2 cdr occupancy grid and applies map revision atomically", async () => {
    const mapChannel = makeChannel(
      10,
      "/orb_lidar/map",
      "nav_msgs/msg/OccupancyGrid",
      OCCUPANCY_GRID_SCHEMA,
    );
    const revChannel = makeChannel(
      11,
      "/orb_lidar/map_revision",
      "orb_slam3_msgs/msg/MapRevision",
      MAP_REVISION_SCHEMA,
    );

    const ws = await openAndAdvertise([mapChannel, revChannel]);
    const subs = parseSubscribeOps(ws);
    const mapSub = subs.find((s) => s.channelId === 10)!;
    const revSub = subs.find((s) => s.channelId === 11)!;

    const gridMsg = {
      header: { stamp: { sec: 10, nanosec: 0 }, frame_id: "orb_map" },
      info: {
        map_load_time: { sec: 10, nanosec: 0 },
        resolution: 0.05,
        width: 2,
        height: 2,
        origin: identityPose(),
      },
      data: [0, 100, -1, 50],
    };
    const cdr = serialize(OCCUPANCY_GRID_SCHEMA, gridMsg);
    ws.deliverMessageData(mapSub.id, 1000n, cdr);

    const revMsg = {
      header: { stamp: { sec: 10, nanosec: 1 }, frame_id: "orb_map" },
      state: 2,
      graph_revision: 7n,
      map_revision: 3n,
      input_scan_count: 100n,
      committed_scan_count: 90n,
      duration_ms: 12.5,
      detail: "ok",
    };
    ws.deliverMessageData(revSub.id, 1001n, serialize(MAP_REVISION_SCHEMA, revMsg));

    const state = await waitForState(
      store,
      (s) => s.map !== undefined && s.mapRevision === 3n,
    );

    expect(state.map).toBeDefined();
    expect(state.map!.info.width).toBe(2);
    expect(state.map!.info.height).toBe(2);
    expect(state.map!.info.resolution).toBeCloseTo(0.05);
    expect(Array.from(state.map!.data)).toEqual([0, 100, -1, 50]);
    expect(state.mapRevision).toBe(3n);
    expect(state.graphRevision).toBe(7n);
  });

  it("accepts newer path revisions and ignores stale ones", async () => {
    const pathChannel = makeChannel(
      20,
      "/orb_lidar/corrected_path_revisioned",
      "orb_slam3_msgs/msg/RevisionedPath",
      REVISIONED_PATH_SCHEMA,
    );
    const ws = await openAndAdvertise([pathChannel]);
    const sub = parseSubscribeOps(ws).find((s) => s.channelId === 20)!;

    const pathRev5 = {
      header: { stamp: { sec: 1, nanosec: 0 }, frame_id: "orb_map" },
      graph_revision: 5n,
      poses: [stampedPose(0, 0), stampedPose(1, 0)],
    };
    ws.deliverMessageData(sub.id, 1n, serialize(REVISIONED_PATH_SCHEMA, pathRev5));

    let state = await waitForState(store, (s) => s.graphRevision === 5n && s.orbPath.length === 2);
    expect(state.orbPath).toHaveLength(2);
    expect(state.orbPath[0]).toMatchObject({ x: 0, y: 0 });
    expect(state.orbPath[1]).toMatchObject({ x: 1, y: 0 });

    // Stale revision 4 must not replace path
    const pathRev4 = {
      header: { stamp: { sec: 2, nanosec: 0 }, frame_id: "orb_map" },
      graph_revision: 4n,
      poses: [stampedPose(9, 9)],
    };
    ws.deliverMessageData(sub.id, 2n, serialize(REVISIONED_PATH_SCHEMA, pathRev4));
    await flushRaf();
    await new Promise((r) => setTimeout(r, 20));

    state = store.getState();
    expect(state.graphRevision).toBe(5n);
    expect(state.orbPath).toHaveLength(2);
    expect(state.orbPath[1]).toMatchObject({ x: 1, y: 0 });

    // Newer revision 6 replaces
    const pathRev6 = {
      header: { stamp: { sec: 3, nanosec: 0 }, frame_id: "orb_map" },
      graph_revision: 6n,
      poses: [stampedPose(0, 0), stampedPose(1, 1), stampedPose(2, 2)],
    };
    ws.deliverMessageData(sub.id, 3n, serialize(REVISIONED_PATH_SCHEMA, pathRev6));
    state = await waitForState(store, (s) => s.graphRevision === 6n && s.orbPath.length === 3);
    expect(state.orbPath).toHaveLength(3);
    expect(state.orbPath[2]).toMatchObject({ x: 2, y: 2 });
  });

  it("decodes tracked frame into tracking summary", async () => {
    const channel = makeChannel(
      30,
      "/orb_slam3/tracked_frame",
      "orb_slam3_msgs/msg/TrackedFrame",
      TRACKED_FRAME_SCHEMA,
    );
    const ws = await openAndAdvertise([channel]);
    const sub = parseSubscribeOps(ws).find((s) => s.channelId === 30)!;

    const frame = {
      header: { stamp: { sec: 5, nanosec: 0 }, frame_id: "orb_map" },
      tracking_state: 2, // OK
      pose_valid: true,
      map_id: 1n,
      reference_keyframe_id: 10n,
      pose: poseAt(1.5, 2.5, 0.1),
      reference_to_frame: {
        translation: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0, w: 1 },
      },
      graph_revision: 9n,
      tracked_keypoints: 120,
    };
    ws.deliverMessageData(sub.id, 1n, serialize(TRACKED_FRAME_SCHEMA, frame));

    const state = await waitForState(store, (s) => s.tracking.state === "OK");
    expect(state.tracking.state).toBe("OK");
    expect(state.tracking.poseValid).toBe(true);
    expect(state.tracking.trackedKeypoints).toBe(120);
    expect(state.tracking.pose).toMatchObject({ x: 1.5, y: 2.5 });
    expect(state.graphRevision).toBe(9n);
  });

  it("caps event history at 2000 entries", async () => {
    const channel = makeChannel(
      40,
      "/orb_slam3/events",
      "orb_slam3_msgs/msg/TrackingEvent",
      TRACKING_EVENT_SCHEMA,
    );
    const ws = await openAndAdvertise([channel]);
    const sub = parseSubscribeOps(ws).find((s) => s.channelId === 40)!;

    for (let i = 0; i < 2010; i++) {
      const ev = {
        header: { stamp: { sec: i, nanosec: 0 }, frame_id: "orb_map" },
        type: 3, // LOOP_CLOSED
        graph_revision: BigInt(i),
        map_id: 1n,
        keyframe_id: BigInt(i),
        detail: `event-${i}`,
      };
      ws.deliverMessageData(sub.id, BigInt(i), serialize(TRACKING_EVENT_SCHEMA, ev));
    }

    const state = await waitForState(store, (s) => s.events.length === 2000);
    expect(state.events).toHaveLength(2000);
    // Oldest of the retained window should be event-10 (0..9 dropped)
    expect(state.events[0]!.detail).toBe("event-10");
    expect(state.events[1999]!.detail).toBe("event-2009");
  });

  it("preserves last display state on disconnect and marks disconnected", async () => {
    const pathChannel = makeChannel(
      50,
      "/orb_lidar/corrected_path_revisioned",
      "orb_slam3_msgs/msg/RevisionedPath",
      REVISIONED_PATH_SCHEMA,
    );
    const ws = await openAndAdvertise([pathChannel]);
    const sub = parseSubscribeOps(ws).find((s) => s.channelId === 50)!;

    const path = {
      header: { stamp: { sec: 1, nanosec: 0 }, frame_id: "orb_map" },
      graph_revision: 2n,
      poses: [stampedPose(3, 4)],
    };
    ws.deliverMessageData(sub.id, 1n, serialize(REVISIONED_PATH_SCHEMA, path));
    await waitForState(store, (s) => s.orbPath.length === 1 && s.connection === "connected");

    ws.close();
    const state = await waitForState(store, (s) => s.connection === "disconnected");

    expect(state.connection).toBe("disconnected");
    expect(state.orbPath).toHaveLength(1);
    expect(state.orbPath[0]).toMatchObject({ x: 3, y: 4 });
    expect(state.graphRevision).toBe(2n);
  });

  it("notifies subscribers with immutable snapshots via rAF batching", async () => {
    const channel = makeChannel(
      60,
      "/orb_lidar/wheel_path",
      "nav_msgs/msg/Path",
      NAV_PATH_SCHEMA,
    );
    // Re-create with wheel path only — need allowed + cdr
    connection.close();
    store = createDashboardStore();
    snapshots = [];
    store.subscribe((s) => snapshots.push(s));
    connection = createDashboardConnection({
      url: "ws://127.0.0.1:8765",
      store,
      webSocketFactory: wsFactory,
    });

    const ws = await openAndAdvertise([channel]);
    const sub = parseSubscribeOps(ws).find((s) => s.channelId === 60)!;

    const pathMsg = {
      header: { stamp: { sec: 1, nanosec: 0 }, frame_id: "odom" },
      poses: [stampedPose(0, 0), stampedPose(1, 0)],
    };
    ws.deliverMessageData(sub.id, 1n, serialize(NAV_PATH_SCHEMA, pathMsg));

    await waitForState(store, (s) => s.wheelPath.length === 2);
    // Allow rAF-batched subscriber notification to flush (getState may win the race).
    await flushRaf();
    await new Promise((r) => setTimeout(r, 0));

    // Snapshots are immutable copies: mutating a returned array must not affect store
    const last = store.getState();
    const prevLen = last.wheelPath.length;
    last.wheelPath.push({ x: 99, y: 99, yaw: 0 });
    expect(store.getState().wheelPath).toHaveLength(prevLen);

    // At least one subscriber notification after connect/path
    expect(snapshots.length).toBeGreaterThan(0);
    const withPath = [...snapshots].reverse().find((s) => s.wheelPath.length === 2);
    expect(withPath).toBeDefined();
    expect(withPath!.wheelPath).toHaveLength(2);
  });

  it("copies occupancy data into a new Int8Array so callers cannot mutate the store", () => {
    const store = createDashboardStore();
    const shared = new Int8Array([0, 100, -1, 50]);
    store.setMap({
      header: { stamp: { sec: 0 }, frame_id: "orb_map" },
      info: {
        resolution: 0.05,
        width: 2,
        height: 2,
        origin: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
      data: shared,
    });
    const snap = store.getState();
    expect(snap.map).toBeDefined();
    expect(snap.map!.data).toBeInstanceOf(Int8Array);
    expect(snap.map!.data).not.toBe(shared);
    expect(Array.from(snap.map!.data)).toEqual([0, 100, -1, 50]);
    // Mutating the original buffer must not affect the stored copy.
    shared[0] = 99;
    expect(Array.from(store.getState().map!.data)).toEqual([0, 100, -1, 50]);
    // Mutating a snapshot must not affect the store either.
    const data = store.getState().map!.data as Int8Array;
    data[1] = 42;
    expect(Array.from(store.getState().map!.data)).toEqual([0, 100, -1, 50]);
  });

  it("ignores map_revision lower than the current (monotonic)", () => {
    const store = createDashboardStore();
    store.setMapRevision(5n, 10n);
    expect(store.getState().mapRevision).toBe(10n);
    expect(store.getState().graphRevision).toBe(5n);

    store.setMapRevision(6n, 8n); // lower map revision — drop map, still allow graph bump
    expect(store.getState().mapRevision).toBe(10n);
    expect(store.getState().graphRevision).toBe(6n);

    store.setMapRevision(6n, 10n); // equal map revision — no-op for map
    expect(store.getState().mapRevision).toBe(10n);

    store.setMapRevision(7n, 11n); // higher — accept
    expect(store.getState().mapRevision).toBe(11n);
    expect(store.getState().graphRevision).toBe(7n);
  });

  it("clears keyframes when MarkerArray is empty", async () => {
    const MARKER_ARRAY_SCHEMA = `
visualization_msgs/Marker[] markers
================================================================================
MSG: visualization_msgs/Marker
std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
builtin_interfaces/Duration lifetime
bool frame_locked
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] colors
string text
string mesh_resource
bool mesh_use_embedded_materials
${HEADER_DEFS}
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: builtin_interfaces/Duration
int32 sec
uint32 nanosec
`;
    const channel = makeChannel(
      80,
      "/orb_slam3/keyframes",
      "visualization_msgs/msg/MarkerArray",
      MARKER_ARRAY_SCHEMA,
    );
    const ws = await openAndAdvertise([channel]);
    const sub = parseSubscribeOps(ws).find((s) => s.channelId === 80)!;

    const withMarkers = {
      markers: [
        {
          header: { stamp: { sec: 1, nanosec: 0 }, frame_id: "orb_map" },
          ns: "kf",
          id: 0,
          type: 1,
          action: 0,
          pose: poseAt(1, 2, 0),
          scale: { x: 1, y: 1, z: 1 },
          color: { r: 1, g: 0, b: 0, a: 1 },
          lifetime: { sec: 0, nanosec: 0 },
          frame_locked: false,
          points: [],
          colors: [],
          text: "",
          mesh_resource: "",
          mesh_use_embedded_materials: false,
        },
      ],
    };
    ws.deliverMessageData(sub.id, 1n, serialize(MARKER_ARRAY_SCHEMA, withMarkers));
    await waitForState(store, (s) => s.keyframes.length === 1);
    expect(store.getState().keyframes[0]).toMatchObject({ x: 1, y: 2 });

    const empty = { markers: [] };
    ws.deliverMessageData(sub.id, 2n, serialize(MARKER_ARRAY_SCHEMA, empty));
    await waitForState(store, (s) => s.keyframes.length === 0);
    expect(store.getState().keyframes).toEqual([]);
  });

  it("revokes previous tracking image object URLs when replaced", async () => {
    const COMPRESSED_IMAGE_SCHEMA = `
std_msgs/Header header
string format
uint8[] data
${HEADER_DEFS}
`;
    const channel = makeChannel(
      70,
      "/orb_slam3/tracking_image/compressed",
      "sensor_msgs/msg/CompressedImage",
      COMPRESSED_IMAGE_SCHEMA,
    );

    // Polyfill URL.createObjectURL / revokeObjectURL for node
    const created: string[] = [];
    const revoked: string[] = [];
    let urlCounter = 0;
    const originalCreate = URL.createObjectURL;
    const originalRevoke = URL.revokeObjectURL;
    URL.createObjectURL = vi.fn((_blob: Blob) => {
      const u = `blob:test-${++urlCounter}`;
      created.push(u);
      return u;
    }) as typeof URL.createObjectURL;
    URL.revokeObjectURL = vi.fn((u: string) => {
      revoked.push(u);
    }) as typeof URL.revokeObjectURL;

    try {
      const ws = await openAndAdvertise([channel]);
      const sub = parseSubscribeOps(ws).find((s) => s.channelId === 70)!;

      const img1 = {
        header: { stamp: { sec: 1, nanosec: 0 }, frame_id: "cam" },
        format: "jpeg",
        data: new Uint8Array([0xff, 0xd8, 0xff, 0xd9]),
      };
      ws.deliverMessageData(sub.id, 1n, serialize(COMPRESSED_IMAGE_SCHEMA, img1));
      await waitForState(store, (s) => s.trackingImageUrl === "blob:test-1");

      const img2 = {
        header: { stamp: { sec: 2, nanosec: 0 }, frame_id: "cam" },
        format: "jpeg",
        data: new Uint8Array([0xff, 0xd8, 0x00, 0xd9]),
      };
      ws.deliverMessageData(sub.id, 2n, serialize(COMPRESSED_IMAGE_SCHEMA, img2));
      await waitForState(store, (s) => s.trackingImageUrl === "blob:test-2");

      expect(revoked).toContain("blob:test-1");
      expect(store.getState().trackingImageUrl).toBe("blob:test-2");
    } finally {
      URL.createObjectURL = originalCreate;
      URL.revokeObjectURL = originalRevoke;
    }
  });
});
