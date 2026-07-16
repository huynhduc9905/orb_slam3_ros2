import { test, expect, Page } from "@playwright/test";

const MAP_PNG = "iVBORw0KGgoAAAANSUhEUgAAAAQAAAAECAIAAAAmkwkpAAAAFElEQVR4nGMM0NjCAANMDEgANwcAOzgBNCn+UqsAAAAASUVORK5CYII=";
const OLD_MAP_PNG = "iVBORw0KGgoAAAANSUhEUgAAAAQAAAAECAIAAAAmkwkpAAAAFElEQVR4nGM8ISfHAANMDEgANwcANHYBDK9qubUAAAAASUVORK5CYII=";
const NEW_MAP_PNG = "iVBORw0KGgoAAAANSUhEUgAAAAQAAAAECAIAAAAmkwkpAAAAFElEQVR4nGMU2XKHAQaYGJAAbg4AUkQBrG7pmEEAAAAASUVORK5CYII=";
const viewports = [{ width: 1440, height: 900 }, { width: 1024, height: 768 }, { width: 390, height: 844 }];

function state(kind: string) {
  const base = {
    state: "OK", tracking: { state: "OK", pose: { x: 0, y: 0, yaw: 0 }, keypoints: 42, hz: 10 },
    odom: { hz: 20 }, map: { resolution: 0.1, origin_x: -0.5, origin_y: -0.5, width: 10, height: 10, revision: 7, state: "PUBLISHED", graph_revision: 12 },
    map_revision: { state: "PUBLISHED", graph_revision: 12, map_revision: 7 },
    paths: { corrected: [], wheel: [], provisional: [] }, fallback: { active: false, pose: null, trail: [] }, events: [],
  };
  if (kind === "lost") return { ...base, state: "LOST", tracking: { ...base.tracking, state: "LOST" }, fallback: { active: true, pose: { x: 0, y: 0, yaw: 0 }, trail: [{ x: -0.2, y: -0.2 }, { x: 0.2, y: 0.2 }] } };
  if (kind === "recovery") return { ...base, state: "recovery_pending", tracking: { ...base.tracking, state: "recovery_pending" }, fallback: { active: true, pose: { x: 0, y: 0, yaw: 0 }, trail: [{ x: -0.2, y: -0.2 }, { x: 0.2, y: 0.2 }] }, paths: { ...base.paths, provisional: [[0.3, -0.3]] } };
  if (kind === "corrected") return { ...base, paths: { ...base.paths, corrected: [{ x: -0.3, y: -0.3, yaw: 0 }, { x: 0.35, y: 0.35, yaw: 0 }] } };
  return base;
}

async function serveFixture(page: Page, kind: string) {
  await page.route("**/state", async route => {
    if (kind === "disconnected") return route.abort();
    await route.fulfill({ status: 200, contentType: "application/json", body: JSON.stringify(state(kind)) });
  });
  await page.route("**/map.png*", route => route.fulfill({ status: 200, contentType: "image/png", body: Buffer.from(MAP_PNG, "base64") }));
  await page.goto("/");
  await page.waitForTimeout(150);
  await expect(page.locator("#map")).toBeVisible();
  await expect(page.locator("#revisions")).toHaveText(/g\d+ \/ m\d+/);
  expect(await page.locator("body").evaluate(el => el.scrollWidth <= el.clientWidth)).toBe(true);
}

for (const viewport of viewports) {
  for (const kind of ["healthy", "lost", "recovery", "corrected", "disconnected"]) {
    test(`${kind} at ${viewport.width}x${viewport.height}`, async ({ page }) => {
      await page.setViewportSize(viewport);
      await serveFixture(page, kind);
      if (viewport.width === 390) {
        const layout = await page.evaluate(() => {
          const box = (selector: string) => {
            const rect = document.querySelector(selector)!.getBoundingClientRect();
            return { left: rect.left, right: rect.right, top: rect.top, bottom: rect.bottom, width: rect.width, height: rect.height };
          };
          return {
            map: box("#map"), mapRegion: box(".map-region"), side: box(".side"),
            header: box(".status-bar"), brand: box(".brand"), revisions: box("#revisions"), track: box("#track"), conn: box("#conn"),
            visibleHeaderChildren: [...document.querySelectorAll<HTMLElement>(".status-bar > *")]
              .filter(element => getComputedStyle(element).display !== "none" && !element.hidden)
              .map(element => {
                const rect = element.getBoundingClientRect();
                return { id: element.id || element.className, left: rect.left, right: rect.right,
                  top: rect.top, bottom: rect.bottom, width: rect.width, height: rect.height,
                  scrollWidth: element.scrollWidth, clientWidth: element.clientWidth };
              }),
          };
        });
        expect(layout.map.width).toBeGreaterThanOrEqual(360);
        expect(layout.brand.width).toBeGreaterThanOrEqual(50);
        expect(layout.side.top).toBeGreaterThanOrEqual(layout.mapRegion.bottom - 1);
        expect(layout.side.left).toBeLessThanOrEqual(1);
        expect(layout.side.width).toBeGreaterThanOrEqual(380);
        expect(layout.side.height).toBe(130);
        expect(layout.revisions.right).toBeLessThanOrEqual(390);
        expect(layout.revisions.top).toBeGreaterThanOrEqual(layout.header.top);
        for (const element of [layout.track, layout.conn, layout.revisions]) {
          expect(element.top).toBeGreaterThanOrEqual(layout.header.top);
          expect(element.bottom).toBeLessThanOrEqual(layout.header.bottom);
        }
        expect(layout.revisions.left).toBeGreaterThan(layout.track.right);
        let previousRight = layout.header.left;
        for (const child of layout.visibleHeaderChildren) {
          expect(child.left).toBeGreaterThanOrEqual(0);
          expect(child.right).toBeLessThanOrEqual(390);
          expect(child.top).toBeGreaterThanOrEqual(layout.header.top);
          expect(child.bottom).toBeLessThanOrEqual(layout.header.bottom);
          expect(child.scrollWidth).toBeLessThanOrEqual(child.clientWidth + 1);
          expect(child.left).toBeGreaterThanOrEqual(previousRight - 1);
          previousRight = child.right;
        }
        expect(await page.locator(".brand").textContent()).toBe("ORB Lidar");
        await expect(page.locator(".brand")).toBeVisible();
        await expect(page.locator(".brand")).toHaveText("ORB Lidar");
        if (kind === "recovery") {
          await expect(page.locator("#recovery")).toBeVisible();
          await expect(page.locator("#recovery")).toHaveText("RECOVERY");
        }
        if (kind === "disconnected") {
          await expect(page.locator("#conn")).toBeVisible();
          await expect(page.locator("#conn")).toHaveText("disconnected");
        }
        if (kind === "lost" || kind === "recovery") {
          await page.screenshot({ path: `test-results/full-${kind}-390x844.png`, fullPage: true });
        }
      }
      if (kind === "disconnected") {
        await expect(page.locator("#conn")).toHaveText("disconnected");
        return;
      }
      await expect(page.locator("#kp")).toHaveText("42");
      await page.locator("#map").screenshot({ path: `test-results/${kind}-${viewport.width}x${viewport.height}.png` });
      const pixels = await page.locator("#map").evaluate((canvas: HTMLCanvasElement) => {
        const data = canvas.getContext("2d")!.getImageData(0, 0, canvas.width, canvas.height).data;
        let yellow = 0, mapSpecific = 0, blue = 0, green = 0;
        for (let i = 0; i < data.length; i += 4) {
          const [r, g, b, a] = data.slice(i, i + 4);
          if (a && Math.abs(r - 80) < 8 && Math.abs(g - 40) < 8 && Math.abs(b - 180) < 8) mapSpecific++;
          if (r > 180 && g > 140 && b < 100) yellow++;
          if (b > 120 && g > 80 && r < 120) blue++;
          if (g > 150 && r < 80 && b < 140) green++;
        }
        const endpoint = (() => {
          const worldW = 1, worldH = 1, scale = Math.min(canvas.width / worldW, canvas.height / worldH) * 0.95;
          const offX = (canvas.width - worldW * scale) / 2, offY = (canvas.height - worldH * scale) / 2;
          return [offX + 0.85 * scale, offY + 0.15 * scale];
        })();
        const provisional = (() => {
          const scale = Math.min(canvas.width, canvas.height) * 0.95;
          const offX = (canvas.width - scale) / 2, offY = (canvas.height - scale) / 2;
          return [offX + 0.8 * scale, offY + 0.8 * scale];
        })();
        const countNear = (xy: number[], color: "blue" | "yellow") => {
          let count = 0;
          const radius = 16;
          for (let y = Math.max(0, Math.floor(xy[1] - radius)); y <= Math.min(canvas.height - 1, Math.ceil(xy[1] + radius)); y++) {
            for (let x = Math.max(0, Math.floor(xy[0] - radius)); x <= Math.min(canvas.width - 1, Math.ceil(xy[0] + radius)); x++) {
              const i = (y * canvas.width + x) * 4, r = data[i], g = data[i + 1], b = data[i + 2];
              if (color === "blue" ? b > 120 && g > 80 && r < 120 : r > 180 && g > 140 && b < 100) count++;
            }
          }
          return count;
        };
        return { yellow, mapSpecific, blue, green, endpointBlue: countNear(endpoint, "blue"), provisionalYellow: countNear(provisional, "yellow") };
      });
      expect(pixels.mapSpecific).toBeGreaterThan(20);
      expect(pixels.green).toBeGreaterThan(2);
      if (kind === "lost") {
        expect(state("lost").paths.provisional).toHaveLength(0);
        expect(pixels.yellow).toBeGreaterThan(2);
      }
      if (kind === "recovery") expect(pixels.provisionalYellow).toBeGreaterThan(2);
      if (kind === "corrected") expect(pixels.endpointBlue).toBeGreaterThan(2);
    });
  }
}

test("out-of-order map loads cannot replace the newest visible revision", async ({ page }) => {
  const requests: number[] = [];
  let stateCalls = 0;
  await page.route("**/state", route => {
    const revision = stateCalls++ === 0 ? 1 : 2;
    const value = state("healthy");
    value.tracking.pose = null;
    value.map.revision = revision;
    value.map_revision.map_revision = revision;
    return route.fulfill({ status: 200, contentType: "application/json", body: JSON.stringify(value) });
  });
  await page.route("**/map.png*", async route => {
    const revision = Number(new URL(route.request().url()).searchParams.get("rev"));
    requests.push(revision);
    await new Promise(resolve => setTimeout(resolve, revision === 1 ? 350 : 20));
    await route.fulfill({ status: 200, contentType: "image/png", body: Buffer.from(revision === 1 ? OLD_MAP_PNG : NEW_MAP_PNG, "base64") });
  });
  await page.goto("/");
  await expect.poll(() => requests.includes(1) && requests.includes(2)).toBe(true);
  await page.waitForTimeout(500);
  const colors = await page.locator("#map").evaluate((canvas: HTMLCanvasElement) => {
    const data = canvas.getContext("2d")!.getImageData(0, 0, canvas.width, canvas.height).data;
    let oldPixels = 0, newPixels = 0;
    for (let i = 0; i < data.length; i += 4) {
      if (data[i] > 170 && data[i + 1] < 60 && data[i + 2] < 60) oldPixels++;
      if (data[i] < 60 && data[i + 1] > 150 && data[i + 2] > 180) newPixels++;
    }
    return { oldPixels, newPixels };
  });
  expect(colors.newPixels).toBeGreaterThan(20);
  expect(colors.oldPixels).toBe(0);
});
