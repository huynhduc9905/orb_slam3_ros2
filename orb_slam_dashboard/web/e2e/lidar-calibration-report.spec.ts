import { execFileSync } from 'node:child_process';
import { readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

import { expect, test } from '@playwright/test';

test('actual calibration report renders at every required viewport', async ({ page }) => {
  const output = join(tmpdir(), `task5-calibration-report-${process.pid}-${Date.now()}`);
  const reportTest = resolve(process.cwd(), '../../build/orb_lidar_mapper/calibration_report_test');
  try {
    execFileSync(reportTest, ['--gtest_filter=CalibrationReport.WritesSelfContainedOutputsAtomically'], {
      env: { ...process.env, TASK5_REPORT_FIXTURE_DIR: output },
      stdio: 'pipe',
    });
    const calibration = JSON.parse(readFileSync(join(output, 'calibration.json'), 'utf8')) as {
      recorded_mount: { x_m: number };
      aggregate: { classification: string; consensus_offset_m: number };
      methods: Array<{ method: string }>;
    };
    const consoleErrors: string[] = [];
    const pageErrors: string[] = [];
    const requests: string[] = [];
    page.on('console', message => { if (message.type() === 'error') consoleErrors.push(message.text()); });
    page.on('pageerror', error => pageErrors.push(error.message));
    page.on('request', request => requests.push(request.url()));
    await page.goto(pathToFileURL(join(output, 'report.html')).href, { waitUntil: 'load' });
    const requestsAfterLoad = requests.length;
    await page.waitForTimeout(100);
    expect(consoleErrors).toEqual([]);
    expect(pageErrors).toEqual([]);
    expect(requests.length).toBe(requestsAfterLoad);
    await expect(page.getByText(calibration.aggregate.classification)).toBeVisible();
    await expect(page.locator('#recorded')).toHaveText(calibration.recorded_mount.x_m.toFixed(3));
    await expect(page.locator('#estimated')).toHaveText(calibration.aggregate.consensus_offset_m.toFixed(3));
    for (const method of calibration.methods) await expect(page.getByText(method.method, { exact: true })).toBeVisible();
    for (const id of ['center-scatter', 'sharpness', 'recorded-map', 'estimated-map']) {
      const canvas = page.locator(`#${id}`);
      await expect(canvas).toBeVisible();
      const nonblank = await canvas.evaluate(element => {
        const canvas = element as HTMLCanvasElement;
        const context = canvas.getContext('2d');
        if (!context) return false;
        return Array.from(context.getImageData(0, 0, canvas.width, canvas.height).data).some(value => value !== 0);
      });
      expect(nonblank).toBe(true);
    }
    const layout = await page.evaluate(() => {
      const sections = Array.from(document.querySelectorAll('section')).map(element => {
        const rect = element.getBoundingClientRect();
        return { left: rect.left, right: rect.right, top: rect.top, bottom: rect.bottom };
      });
      return { scrollWidth: document.documentElement.scrollWidth, viewport: window.innerWidth, sections };
    });
    expect(layout.scrollWidth).toBeLessThanOrEqual(layout.viewport + 1);
    for (const section of layout.sections) {
      expect(section.left).toBeGreaterThanOrEqual(-1);
      expect(section.right).toBeLessThanOrEqual(layout.viewport + 1);
    }
    for (let index = 1; index < layout.sections.length; ++index) {
      expect(layout.sections[index - 1].bottom).toBeLessThanOrEqual(layout.sections[index].top + 1);
    }
  } finally {
    rmSync(output, { recursive: true, force: true });
  }
});
