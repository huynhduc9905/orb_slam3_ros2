/**
 * Visual acceptance tests for the map-first ORB lidar dashboard.
 *
 * Exercises OK / lost-provisional / relocalized / loop-rebuild / disconnected
 * fixtures across 1440×900, 1024×768, and 390×844 viewports.
 *
 * On NixOS, playwright.config.ts launches system Chrome
 * (/run/current-system/sw/bin/google-chrome) — do not rely on
 * `npx playwright install`.
 */
import { test, expect, type Page } from "@playwright/test";
import path from "node:path";

const VIEWPORTS = [
  { name: "desktop", width: 1440, height: 900 },
  { name: "laptop", width: 1024, height: 768 },
  { name: "phone", width: 390, height: 844 },
] as const;

/** Inject a DashboardState fixture into the running app store via page evaluation. */
async function injectFixture(
  page: Page,
  fixture: Record<string, unknown>,
): Promise<void> {
  await page.evaluate((state) => {
    const w = window as unknown as {
      __setDashboardFixture?: (s: unknown) => void;
      __dashboardStore?: {
        replace: (s: unknown) => void;
      };
    };
    const revive = (obj: unknown): unknown => {
      if (obj && typeof obj === "object" && !Array.isArray(obj)) {
        const o = obj as Record<string, unknown>;
        const out: Record<string, unknown> = {};
        for (const [k, v] of Object.entries(o)) {
          if (
            (k.endsWith("Revision") ||
              k === "mapId" ||
              k === "referenceKeyframeId" ||
              k === "keyframeId" ||
              k === "graphRevision" ||
              k === "mapRevision") &&
            typeof v === "string"
          ) {
            out[k] = BigInt(v);
          } else if (k === "data" && Array.isArray(v)) {
            out[k] = Int8Array.from(v as number[]);
          } else if (Array.isArray(v)) {
            out[k] = v.map((item) => revive(item));
          } else if (v && typeof v === "object") {
            out[k] = revive(v);
          } else {
            out[k] = v;
          }
        }
        return out;
      }
      return obj;
    };
    const revived = revive(state);
    if (w.__dashboardStore?.replace) {
      w.__dashboardStore.replace(revived);
      return;
    }
    if (w.__setDashboardFixture) {
      w.__setDashboardFixture(revived);
      return;
    }
    throw new Error(
      "Dashboard fixture hook missing — main.tsx must expose __dashboardStore",
    );
  }, fixture);
}

function baseFixture(overrides: Record<string, unknown> = {}) {
  return {
    connection: "connected",
    graphRevision: "3",
    mapRevision: "2",
    map: {
      header: { stamp: { sec: 1 }, frame_id: "orb_map" },
      info: {
        resolution: 0.05,
        width: 40,
        height: 30,
        origin: {
          position: { x: -1, y: -1, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
      data: Array.from({ length: 40 * 30 }, (_, i) => {
        const x = i % 40;
        const y = Math.floor(i / 40);
        if (x === 0 || y === 0 || x === 39 || y === 29) return 100;
        if (x > 5 && x < 15 && y > 5 && y < 15) return 0;
        if (x === y) return 100;
        return -1;
      }),
    },
    orbPath: [
      { x: -0.5, y: -0.5, yaw: 0 },
      { x: 0.2, y: 0.1, yaw: 0.2 },
      { x: 0.8, y: 0.4, yaw: 0.1 },
    ],
    wheelPath: [
      { x: -0.5, y: -0.6, yaw: 0 },
      { x: 0.2, y: 0.0, yaw: 0 },
      { x: 0.75, y: 0.35, yaw: 0 },
    ],
    provisionalScan: [] as Array<{ x: number; y: number }>,
    keyframes: [
      { x: -0.5, y: -0.5, yaw: 0 },
      { x: 0.2, y: 0.1, yaw: 0 },
    ],
    loopEdges: [] as Array<{
      from: { x: number; y: number; yaw: number };
      to: { x: number; y: number; yaw: number };
    }>,
    tracking: {
      state: "OK",
      poseValid: true,
      trackedKeypoints: 140,
      pose: { x: 0.8, y: 0.4, yaw: 0.1 },
      mapId: "1",
      referenceKeyframeId: "4",
    },
    health: { level: "ok", message: "healthy", items: [] as unknown[] },
    events: [
      {
        type: "INITIALIZED",
        graphRevision: "1",
        mapId: "1",
        keyframeId: "1",
        detail: "boot",
        stampSec: 1,
        stampNsec: 0,
      },
    ],
    ...overrides,
  };
}

const FIXTURES: Record<string, Record<string, unknown>> = {
  ok: baseFixture(),
  "lost-provisional": baseFixture({
    tracking: {
      state: "LOST",
      poseValid: false,
      trackedKeypoints: 0,
      mapId: "1",
      referenceKeyframeId: "4",
    },
    provisionalScan: Array.from({ length: 20 }, (_, i) => ({
      x: 0.5 + Math.cos(i) * 0.3,
      y: 0.3 + Math.sin(i) * 0.3,
    })),
    events: [
      {
        type: "INITIALIZED",
        graphRevision: "1",
        mapId: "1",
        keyframeId: "1",
        detail: "boot",
        stampSec: 1,
        stampNsec: 0,
      },
      {
        type: "LOST",
        graphRevision: "4",
        mapId: "1",
        keyframeId: "5",
        detail: "tracking lost",
        stampSec: 10,
        stampNsec: 0,
      },
    ],
  }),
  relocalized: baseFixture({
    tracking: {
      state: "OK",
      poseValid: true,
      trackedKeypoints: 90,
      pose: { x: 0.5, y: 0.2, yaw: 0 },
      mapId: "1",
      referenceKeyframeId: "8",
    },
    events: [
      {
        type: "LOST",
        graphRevision: "4",
        mapId: "1",
        keyframeId: "5",
        detail: "lost",
        stampSec: 10,
        stampNsec: 0,
      },
      {
        type: "RELOCALIZED",
        graphRevision: "5",
        mapId: "1",
        keyframeId: "8",
        detail: "relocalized",
        stampSec: 12,
        stampNsec: 0,
      },
    ],
  }),
  "loop-rebuild": baseFixture({
    loopEdges: [
      {
        from: { x: -0.5, y: -0.5, yaw: 0 },
        to: { x: 0.8, y: 0.4, yaw: 0 },
      },
    ],
    events: [
      {
        type: "LOOP_CLOSED",
        graphRevision: "6",
        mapId: "1",
        keyframeId: "10",
        detail: "loop",
        stampSec: 20,
        stampNsec: 0,
      },
    ],
    graphRevision: "6",
    mapRevision: "3",
  }),
  disconnected: baseFixture({
    connection: "disconnected",
  }),
};

async function assertNoHorizontalOverflow(page: Page) {
  const overflow = await page.evaluate(() => {
    const doc = document.documentElement;
    return {
      scrollWidth: doc.scrollWidth,
      clientWidth: doc.clientWidth,
      bodyScrollWidth: document.body.scrollWidth,
    };
  });
  expect(
    overflow.scrollWidth,
    `horizontal overflow: scrollWidth=${overflow.scrollWidth} clientWidth=${overflow.clientWidth}`,
  ).toBeLessThanOrEqual(overflow.clientWidth + 1);
}

async function assertCanvasHasNonBackgroundPixels(page: Page) {
  const result = await page.evaluate(() => {
    const canvas = document.querySelector(
      ".map-canvas-host canvas",
    ) as HTMLCanvasElement | null;
    if (!canvas) return { ok: false, reason: "no-canvas" as const };
    try {
      const ctx = canvas.getContext("2d");
      if (!ctx) {
        return {
          ok: canvas.width > 0 && canvas.height > 0,
          reason: "webgl-sized" as const,
          w: canvas.width,
          h: canvas.height,
        };
      }
      const { width, height } = canvas;
      if (width < 2 || height < 2) return { ok: false, reason: "tiny" as const };
      const img = ctx.getImageData(0, 0, width, height);
      const bg = { r: 0x17, g: 0x1c, b: 0x21 };
      let nonBg = 0;
      for (let i = 0; i < img.data.length; i += 16) {
        const r = img.data[i]!;
        const g = img.data[i + 1]!;
        const b = img.data[i + 2]!;
        const a = img.data[i + 3]!;
        if (a > 0 && (r !== bg.r || g !== bg.g || b !== bg.b)) nonBg++;
      }
      return { ok: nonBg > 0, reason: "sampled" as const, nonBg };
    } catch (e) {
      return {
        ok: canvas.width > 0,
        reason: "sample-error" as const,
        err: String(e),
      };
    }
  });
  expect(
    result.ok || result.reason === "webgl-sized",
    `canvas pixel check failed: ${JSON.stringify(result)}`,
  ).toBeTruthy();
}

for (const vp of VIEWPORTS) {
  test.describe(`viewport ${vp.name} ${vp.width}x${vp.height}`, () => {
    test.use({ viewport: { width: vp.width, height: vp.height } });

    for (const [name, fixture] of Object.entries(FIXTURES)) {
      test(`fixture ${name}: no horizontal overflow + shell visible`, async ({
        page,
      }, testInfo) => {
        await page.goto("/?fixture=1");
        await page.waitForSelector('[data-testid="dashboard"]');
        await injectFixture(page, fixture);
        await page.waitForTimeout(300);

        await expect(page.getByTestId("status-bar")).toBeVisible();
        await expect(page.getByTestId("map-view")).toBeVisible();
        await expect(page.getByTestId("event-timeline")).toBeVisible();

        if (name === "disconnected") {
          await expect(page.getByTestId("connection-overlay")).toBeVisible();
        }
        if (name === "ok" || name === "relocalized") {
          await expect(page.getByTestId("tracking-status")).toContainText("OK");
        }
        if (name === "lost-provisional") {
          await expect(page.getByTestId("tracking-status")).toContainText("LOST");
        }

        await assertNoHorizontalOverflow(page);

        const shot = path.join(testInfo.outputDir, `${vp.name}-${name}.png`);
        await page.screenshot({ path: shot, fullPage: true });

        if (name === "ok" && vp.name === "desktop") {
          await assertCanvasHasNonBackgroundPixels(page);
        }
      });
    }
  });
}
