import type { DashboardState } from "../types";

function trackingClass(state: string): string {
  switch (state) {
    case "OK":
      return "status-ok";
    case "RECENTLY_LOST":
      return "status-recently-lost status-warn";
    case "LOST":
      return "status-lost status-error";
    default:
      return "status-unknown";
  }
}

function connectionClass(c: DashboardState["connection"]): string {
  switch (c) {
    case "connected":
      return "status-ok";
    case "connecting":
      return "status-connecting";
    case "error":
      return "status-error";
    default:
      return "status-disconnected";
  }
}

export interface StatusBarProps {
  state: DashboardState;
}

export function StatusBar({ state }: StatusBarProps) {
  const t = state.tracking;
  return (
    <header className="status-bar" data-testid="status-bar">
      <span className="brand">ORB Lidar</span>
      <span
        className={`status-badge ${connectionClass(state.connection)}`}
        data-testid="connection-status"
      >
        {state.connection}
      </span>
      <span
        className={`status-badge ${trackingClass(t.state)}`}
        data-testid="tracking-status"
      >
        {t.state}
      </span>
      <span className="meta" data-testid="keypoints">
        kp {t.trackedKeypoints}
      </span>
      <span className="meta" data-testid="revisions">
        g{state.graphRevision.toString()} / m{state.mapRevision.toString()}
      </span>
      <span className="spacer" />
      <span className="meta" data-testid="health-summary">
        health: {state.health.level}
      </span>
    </header>
  );
}
