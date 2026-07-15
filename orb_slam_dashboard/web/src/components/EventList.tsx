import type { DashboardEvent } from "../types";

export interface EventListProps {
  events: DashboardEvent[];
}

/** Newest-first event list (read-only). */
export function EventList({ events }: EventListProps) {
  const ordered = [...events].reverse();
  return (
    <section className="panel-section" data-testid="event-list-section">
      <h2>Events</h2>
      <ul className="event-list" data-testid="event-list">
        {ordered.length === 0 ? (
          <li className="meta">No events yet</li>
        ) : (
          ordered.map((ev, i) => (
            <li key={`${ev.stampSec}-${ev.stampNsec}-${i}`} data-testid="event-item">
              <span className="event-type">{ev.type}</span>
              <span className="event-detail">{ev.detail}</span>
            </li>
          ))
        )}
      </ul>
    </section>
  );
}
