/**
 * @vitest-environment jsdom
 */
import { describe, expect, it, beforeEach, afterEach } from "vitest";
import { render, screen, fireEvent, cleanup, within } from "@testing-library/react";
import React from "react";

import { App } from "../src/App";
import { createDashboardStore } from "../src/store";
import {
  createInitialDashboardState,
  type DashboardState,
  type DashboardEvent,
} from "../src/types";

function makeState(partial: Partial<DashboardState> = {}): DashboardState {
  const base = createInitialDashboardState();
  return {
    ...base,
    ...partial,
    tracking: { ...base.tracking, ...partial.tracking },
    health: {
      ...base.health,
      ...partial.health,
      items: partial.health?.items ?? base.health.items,
    },
  };
}

function renderApp(state: DashboardState) {
  const store = createDashboardStore(state);
  return {
    store,
    ...render(<App store={store} connectOnMount={false} />),
  };
}

beforeEach(() => {
  Object.defineProperty(window, "innerWidth", { writable: true, value: 1440 });
  Object.defineProperty(window, "innerHeight", { writable: true, value: 900 });
});

afterEach(() => {
  cleanup();
});

describe("StatusBar tracking colors", () => {
  it("shows OK status with success color class", () => {
    renderApp(
      makeState({
        connection: "connected",
        tracking: {
          state: "OK",
          poseValid: true,
          trackedKeypoints: 100,
          mapId: 1n,
          referenceKeyframeId: 2n,
        },
      }),
    );
    const badge = screen.getByTestId("tracking-status");
    expect(badge.textContent).toContain("OK");
    expect(badge.className).toMatch(/status-ok/);
  });

  it("shows RECENTLY_LOST with warn color class", () => {
    renderApp(
      makeState({
        connection: "connected",
        tracking: {
          state: "RECENTLY_LOST",
          poseValid: false,
          trackedKeypoints: 0,
          mapId: 1n,
          referenceKeyframeId: 2n,
        },
      }),
    );
    const badge = screen.getByTestId("tracking-status");
    expect(badge.textContent).toContain("RECENTLY_LOST");
    expect(badge.className).toMatch(/status-recently-lost|status-warn/);
  });

  it("shows LOST with error color class", () => {
    renderApp(
      makeState({
        connection: "connected",
        tracking: {
          state: "LOST",
          poseValid: false,
          trackedKeypoints: 0,
          mapId: 1n,
          referenceKeyframeId: 2n,
        },
      }),
    );
    const badge = screen.getByTestId("tracking-status");
    expect(badge.textContent).toContain("LOST");
    expect(badge.className).toMatch(/status-lost|status-error/);
  });
});

describe("layer toggles", () => {
  it("toggles map layers via Eye/EyeOff controls with aria-labels", () => {
    renderApp(makeState({ connection: "connected" }));

    const orbToggle = screen.getByRole("button", { name: /orb path/i });
    expect(orbToggle.getAttribute("aria-pressed")).toBe("true");
    fireEvent.click(orbToggle);
    expect(orbToggle.getAttribute("aria-pressed")).toBe("false");

    const wheelToggle = screen.getByRole("button", { name: /wheel path/i });
    fireEvent.click(wheelToggle);
    expect(wheelToggle.getAttribute("aria-pressed")).toBe("false");

    const scanToggle = screen.getByRole("button", { name: /provisional scan/i });
    fireEvent.click(scanToggle);
    expect(scanToggle.getAttribute("aria-pressed")).toBe("false");

    fireEvent.click(orbToggle);
    expect(orbToggle.getAttribute("aria-pressed")).toBe("true");
  });

  it("exposes zoom and fit controls with aria-label and title tooltips", () => {
    renderApp(makeState({ connection: "connected" }));
    for (const name of [/zoom in/i, /zoom out/i, /fit map/i]) {
      const btn = screen.getByRole("button", { name });
      expect(btn.getAttribute("aria-label")).toBeTruthy();
      expect(btn.getAttribute("title")).toBeTruthy();
    }
  });
});

describe("event ordering", () => {
  it("lists events newest-first", () => {
    const events: DashboardEvent[] = [
      {
        type: "INITIALIZED",
        graphRevision: 1n,
        mapId: 1n,
        keyframeId: 1n,
        detail: "first",
        stampSec: 1,
        stampNsec: 0,
      },
      {
        type: "LOOP_CLOSED",
        graphRevision: 2n,
        mapId: 1n,
        keyframeId: 2n,
        detail: "second",
        stampSec: 2,
        stampNsec: 0,
      },
      {
        type: "RELOCALIZED",
        graphRevision: 3n,
        mapId: 1n,
        keyframeId: 3n,
        detail: "third",
        stampSec: 3,
        stampNsec: 0,
      },
    ];
    renderApp(makeState({ connection: "connected", events }));
    const list = screen.getByTestId("event-list");
    const items = within(list).getAllByTestId("event-item");
    expect(items).toHaveLength(3);
    expect(items[0]!.textContent).toContain("RELOCALIZED");
    expect(items[0]!.textContent).toContain("third");
    expect(items[1]!.textContent).toContain("LOOP_CLOSED");
    expect(items[2]!.textContent).toContain("INITIALIZED");
  });
});

describe("stale / disconnected overlay", () => {
  it("shows disconnected overlay with WifiOff when disconnected", () => {
    renderApp(makeState({ connection: "disconnected" }));
    const overlay = screen.getByTestId("connection-overlay");
    expect(overlay.textContent?.toLowerCase()).toMatch(/disconnected/);
    expect(overlay.querySelector("svg")).toBeTruthy();
  });

  it("shows stale health banner when health level is stale", () => {
    renderApp(
      makeState({
        connection: "connected",
        health: { level: "stale", message: "No diagnostics recently", items: [] },
      }),
    );
    expect(screen.getByTestId("health-panel").textContent?.toLowerCase()).toMatch(
      /stale/,
    );
  });

  it("hides disconnected overlay when connected", () => {
    renderApp(makeState({ connection: "connected" }));
    expect(screen.queryByTestId("connection-overlay")).toBeNull();
  });
});

describe("layout structure", () => {
  it("renders fixed status bar, map-first main grid, and timeline", () => {
    renderApp(makeState({ connection: "connected" }));
    expect(screen.getByTestId("status-bar")).toBeTruthy();
    expect(screen.getByTestId("map-view")).toBeTruthy();
    expect(screen.getByTestId("diagnostics-panel")).toBeTruthy();
    expect(screen.getByTestId("event-timeline")).toBeTruthy();
    expect(screen.getByTestId("health-panel")).toBeTruthy();
    expect(screen.getByTestId("tracking-image")).toBeTruthy();
  });

  it("shows waiting status for zero-sized map without removing layout regions", () => {
    renderApp(
      makeState({
        connection: "connected",
        map: {
          header: { stamp: { sec: 0 }, frame_id: "orb_map" },
          info: {
            resolution: 0.05,
            width: 0,
            height: 0,
            origin: {
              position: { x: 0, y: 0, z: 0 },
              orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
          },
          data: [],
        },
      }),
    );
    expect(screen.getByText(/Waiting for committed map/i)).toBeTruthy();
    expect(screen.getByTestId("map-view")).toBeTruthy();
    expect(screen.getByTestId("status-bar")).toBeTruthy();
  });

  it("is read-only: no publish / service / parameter write controls", () => {
    renderApp(makeState({ connection: "connected" }));
    expect(
      screen.queryByRole("button", { name: /publish|call service|set param/i }),
    ).toBeNull();
    expect(document.querySelector("[data-write-control]")).toBeNull();
  });
});
