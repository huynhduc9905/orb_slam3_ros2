import type { DashboardEvent } from "../types";

export interface TimelineProps {
  events: DashboardEvent[];
}

function formatStamp(sec: number, nsec: number): string {
  const ms = Math.floor(nsec / 1e6);
  return `${sec}.${String(ms).padStart(3, "0")}s`;
}

/** Horizontal scrollable event timeline (54px band). */
export function Timeline({ events }: TimelineProps) {
  // Chronological left→right for a timeline feel; list panel is newest-first.
  const ordered = events.slice(-40);
  return (
    <footer className="timeline" data-testid="event-timeline">
      {ordered.length === 0 ? (
        <div className="timeline-item meta">Waiting for events…</div>
      ) : (
        ordered.map((ev, i) => (
          <div
            key={`${ev.stampSec}-${ev.type}-${i}`}
            className="timeline-item"
            data-testid="timeline-item"
          >
            <span className="t-type">{ev.type}</span>
            <span className="t-time">{formatStamp(ev.stampSec, ev.stampNsec)}</span>
          </div>
        ))
      )}
    </footer>
  );
}
