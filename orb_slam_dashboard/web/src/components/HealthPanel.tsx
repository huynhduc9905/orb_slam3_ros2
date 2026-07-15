import type { HealthSummary } from "../types";

export interface HealthPanelProps {
  health: HealthSummary;
}

export function HealthPanel({ health }: HealthPanelProps) {
  return (
    <section className="panel-section health-panel" data-testid="health-panel">
      <h2>Health</h2>
      <div className={`health-level level-${health.level}`}>{health.level}</div>
      {health.message ? <div className="meta">{health.message}</div> : null}
      {health.items.length > 0 ? (
        <ul className="health-items">
          {health.items.map((item, i) => (
            <li key={`${item.name}-${i}`}>
              <strong>{item.name || "item"}</strong> [{item.level}] {item.message}
            </li>
          ))}
        </ul>
      ) : null}
    </section>
  );
}
