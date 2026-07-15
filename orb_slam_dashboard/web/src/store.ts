import {
  createInitialDashboardState,
  MAX_EVENT_HISTORY,
  type ConnectionState,
  type DashboardEvent,
  type DashboardState,
  type Edge2,
  type HealthSummary,
  type OccupancyGrid,
  type Point2,
  type Pose2,
  type TrackingSummary,
} from "./types";

export type DashboardListener = (state: DashboardState) => void;

export interface DashboardStore {
  getState(): DashboardState;
  subscribe(listener: DashboardListener): () => void;
  setConnection(connection: ConnectionState): void;
  setMap(map: OccupancyGrid): void;
  setMapRevision(graphRevision: bigint, mapRevision: bigint): void;
  setOrbPath(graphRevision: bigint, path: Pose2[]): void;
  setWheelPath(path: Pose2[]): void;
  setProvisionalScan(points: Point2[]): void;
  setKeyframes(poses: Pose2[]): void;
  setLoopEdges(edges: Edge2[]): void;
  setTracking(tracking: TrackingSummary, graphRevision?: bigint): void;
  setHealth(health: HealthSummary): void;
  appendEvent(event: DashboardEvent): void;
  setTrackingImageUrl(url: string | undefined): void;
  /** Replace entire state (tests / reconnect). */
  replace(state: DashboardState): void;
}

function scheduleFrame(cb: () => void): number {
  if (typeof requestAnimationFrame === "function") {
    return requestAnimationFrame(cb);
  }
  // Node / test fallback: coalesce to next macrotask.
  return setTimeout(cb, 0) as unknown as number;
}

/**
 * Immutable dashboard store.
 *
 * Mutations produce a new `DashboardState` object. Subscribers are notified at
 * most once per animation frame so high-rate topics do not thrash React.
 */
export function createDashboardStore(
  initial: DashboardState = createInitialDashboardState(),
): DashboardStore {
  let state: DashboardState = initial;
  const listeners = new Set<DashboardListener>();
  let pendingNotify = false;

  function notify(): void {
    if (pendingNotify) return;
    pendingNotify = true;
    scheduleFrame(() => {
      pendingNotify = false;
      const snap = snapshot();
      for (const listener of listeners) {
        listener(snap);
      }
    });
  }

  function commit(next: DashboardState): void {
    state = next;
    notify();
  }

  /** Shallow-clone mutable arrays so callers cannot mutate store internals. */
  function snapshot(): DashboardState {
    return {
      ...state,
      orbPath: state.orbPath.slice(),
      wheelPath: state.wheelPath.slice(),
      provisionalScan: state.provisionalScan.slice(),
      keyframes: state.keyframes.slice(),
      loopEdges: state.loopEdges.slice(),
      events: state.events.slice(),
      tracking: { ...state.tracking },
      health: { ...state.health, items: state.health.items.slice() },
      map: state.map
        ? {
            ...state.map,
            info: { ...state.map.info },
            // Always copy so snapshot consumers cannot mutate store internals.
            data:
              state.map.data instanceof Int8Array
                ? Int8Array.from(state.map.data)
                : Int8Array.from(state.map.data),
          }
        : undefined,
    };
  }

  return {
    getState(): DashboardState {
      return snapshot();
    },

    subscribe(listener: DashboardListener): () => void {
      listeners.add(listener);
      // Immediate snapshot so UI mounts with current state.
      listener(snapshot());
      return () => {
        listeners.delete(listener);
      };
    },

    setConnection(connection: ConnectionState): void {
      if (state.connection === connection) return;
      commit({ ...state, connection });
    },

    setMap(map: OccupancyGrid): void {
      // Copy occupancy data so CDR buffer views / callers cannot mutate store.
      const dataCopy =
        map.data instanceof Int8Array
          ? Int8Array.from(map.data)
          : Int8Array.from(map.data);
      commit({
        ...state,
        map: {
          ...map,
          info: { ...map.info },
          data: dataCopy,
        },
      });
    },

    setMapRevision(graphRevision: bigint, mapRevision: bigint): void {
      // Atomic revision update; map data arrives on a separate topic.
      // mapRevision is monotonic — ignore lower values (mirrors stale path drop).
      let nextGraph = state.graphRevision;
      if (graphRevision > state.graphRevision) {
        nextGraph = graphRevision;
      }
      let nextMap = state.mapRevision;
      if (mapRevision > state.mapRevision) {
        nextMap = mapRevision;
      }
      if (nextMap === state.mapRevision && nextGraph === state.graphRevision) {
        return;
      }
      commit({
        ...state,
        graphRevision: nextGraph,
        mapRevision: nextMap,
      });
    },

    setOrbPath(graphRevision: bigint, path: Pose2[]): void {
      // Ignore stale path revisions (older than current graph revision).
      if (graphRevision < state.graphRevision) {
        return;
      }
      commit({
        ...state,
        graphRevision,
        orbPath: path,
      });
    },

    setWheelPath(path: Pose2[]): void {
      commit({ ...state, wheelPath: path });
    },

    setProvisionalScan(points: Point2[]): void {
      commit({ ...state, provisionalScan: points });
    },

    setKeyframes(poses: Pose2[]): void {
      commit({ ...state, keyframes: poses });
    },

    setLoopEdges(edges: Edge2[]): void {
      commit({ ...state, loopEdges: edges });
    },

    setTracking(tracking: TrackingSummary, graphRevision?: bigint): void {
      const next: DashboardState = {
        ...state,
        tracking,
      };
      if (graphRevision !== undefined && graphRevision > state.graphRevision) {
        next.graphRevision = graphRevision;
      }
      commit(next);
    },

    setHealth(health: HealthSummary): void {
      commit({ ...state, health });
    },

    appendEvent(event: DashboardEvent): void {
      const events =
        state.events.length >= MAX_EVENT_HISTORY
          ? [...state.events.slice(state.events.length - MAX_EVENT_HISTORY + 1), event]
          : [...state.events, event];
      // Cap strictly at MAX_EVENT_HISTORY.
      const capped =
        events.length > MAX_EVENT_HISTORY
          ? events.slice(events.length - MAX_EVENT_HISTORY)
          : events;
      commit({ ...state, events: capped });
    },

    setTrackingImageUrl(url: string | undefined): void {
      const prev = state.trackingImageUrl;
      if (prev === url) return;
      if (prev !== undefined && typeof URL !== "undefined" && URL.revokeObjectURL) {
        try {
          URL.revokeObjectURL(prev);
        } catch {
          // ignore revoke errors in non-browser envs
        }
      }
      commit({ ...state, trackingImageUrl: url });
    },

    replace(next: DashboardState): void {
      commit(next);
    },
  };
}
