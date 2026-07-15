import { useEffect, useState } from "react";

import { createDashboardConnection } from "./foxglove";
import type { DashboardStore } from "./store";
import type { DashboardState } from "./types";
import { EventList } from "./components/EventList";
import { HealthPanel } from "./components/HealthPanel";
import { MapView } from "./components/MapView";
import { StatusBar } from "./components/StatusBar";
import { Timeline } from "./components/Timeline";
import { TrackingImage } from "./components/TrackingImage";

export interface AppProps {
  store: DashboardStore;
  /** Foxglove WebSocket URL; default from query or localhost. */
  wsUrl?: string;
  /** When false, skip connecting (unit tests). Default true. */
  connectOnMount?: boolean;
}

function defaultWsUrl(): string {
  if (typeof window === "undefined") return "ws://127.0.0.1:8765";
  const q = new URLSearchParams(window.location.search);
  return q.get("ws") ?? `ws://${window.location.hostname}:8765`;
}

/**
 * Map-first read-only dashboard shell.
 * Layout: 46px status · main minmax(0,1fr) 310px · 54px timeline.
 * No publish / service / parameter write controls.
 */
export function App({
  store,
  wsUrl,
  connectOnMount = true,
}: AppProps) {
  const [state, setState] = useState<DashboardState>(() => store.getState());

  useEffect(() => {
    return store.subscribe((s) => setState(s));
  }, [store]);

  useEffect(() => {
    if (!connectOnMount) return;
    const url = wsUrl ?? defaultWsUrl();
    const conn = createDashboardConnection({ url, store });
    conn.connect();
    return () => conn.close();
  }, [store, wsUrl, connectOnMount]);

  return (
    <div className="dashboard" data-testid="dashboard">
      <StatusBar state={state} />
      <div className="main">
        <div className="map-region">
          <MapView state={state} />
        </div>
        <aside className="diagnostics" data-testid="diagnostics-panel">
          <TrackingImage url={state.trackingImageUrl} />
          <HealthPanel health={state.health} />
          <EventList events={state.events} />
        </aside>
      </div>
      <Timeline events={state.events} />
    </div>
  );
}
