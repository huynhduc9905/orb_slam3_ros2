import { execFileSync } from 'node:child_process';
import { readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

import { expect, test } from '@playwright/test';

const colors = {
  odom: [37, 99, 235],
  imu: [22, 163, 74],
  existing: [147, 51, 234],
  rejected: [220, 38, 38],
  coarse: [15, 118, 110],
  refined: [249, 115, 22],
};

function closeToColor(pixel: number[], expected: number[]) {
  return pixel[3] > 0 && expected.every((value, index) => Math.abs(pixel[index] - value) <= 8);
}

test('actual calibration reports render scientific data and fail closed', async ({ page }) => {
  const root = join(tmpdir(), `task5-calibration-report-${process.pid}-${Date.now()}`);
  const normal = join(root, 'normal');
  const inconclusive = join(root, 'inconclusive');
  const reportTest = resolve(process.cwd(), '../../build/orb_lidar_mapper/calibration_report_test');
  const emit = (filter: string, output: string, variable: string) => {
    execFileSync(reportTest, [`--gtest_filter=${filter}`], {
      env: { ...process.env, [variable]: output },
      stdio: 'pipe',
    });
  };
  try {
    emit('CalibrationReport.WritesSelfContainedOutputsAtomically', normal, 'TASK5_REPORT_FIXTURE_DIR');
    emit('CalibrationReport.WritesParseableNullsForInconclusiveScientificValues', inconclusive,
      'TASK5_REPORT_INCONCLUSIVE_FIXTURE_DIR');
    const calibration = JSON.parse(readFileSync(join(normal, 'calibration.json'), 'utf8')) as any;
    const failClosed = JSON.parse(readFileSync(join(inconclusive, 'calibration.json'), 'utf8')) as any;
    expect(failClosed.methods[1].center_x_m).toBeNull();
    expect(failClosed.aggregate.consensus_offset_m).toBeNull();

    const consoleErrors: string[] = [];
    const pageErrors: string[] = [];
    const requests: string[] = [];
    page.on('console', message => { if (message.type() === 'error') consoleErrors.push(message.text()); });
    page.on('pageerror', error => pageErrors.push(error.message));
    page.on('request', request => requests.push(request.url()));
    await page.goto(pathToFileURL(join(normal, 'report.html')).href, { waitUntil: 'load' });
    const requestsAfterLoad = requests.length;
    await page.waitForTimeout(100);
    expect(consoleErrors).toEqual([]);
    expect(pageErrors).toEqual([]);
    expect(requests.length).toBe(requestsAfterLoad);
    expect(await page.locator('button,input,select,textarea,[contenteditable="true"]').count()).toBe(0);
    const html = await page.locator('html').innerHTML();
    expect(html).not.toMatch(/fetch|XMLHttpRequest|WebSocket|localStorage|sessionStorage/);
    await expect(page.getByText(calibration.aggregate.classification)).toBeVisible();
    await expect(page.locator('#recorded')).toHaveText(calibration.recorded_mount.x_m.toFixed(3));
    await expect(page.locator('#estimated')).toHaveText(calibration.aggregate.consensus_offset_m.toFixed(3));
    for (const method of calibration.methods) {
      const row = page.locator('#methods tr', { hasText: method.method });
      await expect(row).toContainText(method.center_x_m.toFixed(3));
      await expect(row).toContainText(method.center_y_m.toFixed(3));
      await expect(row).toContainText(method.forward_offset_m.toFixed(3));
      await expect(row).toContainText(method.delta_from_recorded_m.toFixed(3));
      await expect(row).toContainText(`[${method.confidence_95_m.low_m.toFixed(3)}, ${method.confidence_95_m.high_m.toFixed(3)}]`);
      await expect(row).toContainText(`${method.accepted_pairs}/${method.attempted_pairs}`);
      await expect(row).toContainText(String(method.covered_yaw_sectors));
      await expect(row).toContainText(method.median_rmse_m.toFixed(3));
      await expect(row).toContainText(method.median_overlap.toFixed(3));
    }
    if (await page.evaluate(() => window.innerWidth <= 600)) {
      const mobileRows = await page.locator('#methods tr').evaluateAll(rows => rows.map(row => {
        const rowRect = row.getBoundingClientRect();
        const cells = Array.from(row.querySelectorAll('td')).map(cell => {
          const rect = cell.getBoundingClientRect();
          return {
            label: cell.getAttribute('data-label'),
            text: cell.textContent?.trim() ?? '',
            left: rect.left,
            right: rect.right,
            width: rect.width,
            rowLeft: rowRect.left,
            rowRight: rowRect.right,
          };
        });
        return { cells, rowLeft: rowRect.left, rowRight: rowRect.right };
      }));
      const labels = ['Method', 'Center x', 'Center y', 'Forward offset', 'Delta', '95% CI',
        'Accepted/attempted', 'Sectors', 'RMSE', 'Overlap'];
      expect(await page.locator('#methods').evaluate(table => ({
        scrollWidth: table.scrollWidth,
        clientWidth: table.clientWidth,
      }))).toEqual(expect.objectContaining({ scrollWidth: expect.any(Number), clientWidth: expect.any(Number) }));
      const tableMetrics = await page.locator('#methods').evaluate(table => ({
        scrollWidth: table.scrollWidth,
        clientWidth: table.clientWidth,
      }));
      expect(tableMetrics.scrollWidth).toBeLessThanOrEqual(tableMetrics.clientWidth + 1);
      expect(mobileRows).toHaveLength(calibration.methods.length);
      for (const row of mobileRows) {
        expect(row.cells.map(cell => cell.label)).toEqual(labels);
        expect(row.cells).toHaveLength(labels.length);
        for (const cell of row.cells) {
          expect(cell.text).not.toBe('');
          expect(cell.left).toBeGreaterThanOrEqual(row.rowLeft - 1);
          expect(cell.right).toBeLessThanOrEqual(row.rowRight + 1);
          expect(cell.width).toBeLessThanOrEqual(row.rowRight - row.rowLeft + 1);
        }
      }
    }
    for (const label of ['Odom', 'IMU', 'Existing /scan', 'Recorded center']) {
      await expect(page.locator('#center-legend')).toContainText(label);
    }

    const probes = await page.evaluate(({ colors, calibration }) => {
      const rgb = (id: string, x: number, y: number) => {
        const canvas = document.getElementById(id) as HTMLCanvasElement;
        const data = canvas.getContext('2d')!.getImageData(x, y, 1, 1).data;
        return [data[0], data[1], data[2], data[3]];
      };
      const near = (id: string, x: number, y: number, expected: number[]) => {
        const canvas = document.getElementById(id) as HTMLCanvasElement;
        const data = canvas.getContext('2d')!.getImageData(Math.max(0, x - 5), Math.max(0, y - 5), 11, 11).data;
        for (let i = 0; i < data.length; i += 4) {
          if (data[i + 3] > 0 && expected.every((v, j) => Math.abs(data[i + j] - v) <= 8)) return true;
        }
        return false;
      };
      const hasColor = (id: string, expected: number[]) => {
        const canvas = document.getElementById(id) as HTMLCanvasElement;
        const data = canvas.getContext('2d')!.getImageData(0, 0, canvas.width, canvas.height).data;
        for (let i = 0; i < data.length; i += 4) {
          if (data[i + 3] > 0 && expected.every((v, j) => Math.abs(data[i + j] - v) <= 8)) return true;
        }
        return false;
      };
      const center = (sample: any) => [50 + (sample.center_x_m - calibration.recorded_mount.x_m + 0.05) * 700,
        310 - (sample.center_y_m + 0.30) * 700];
      const accepted = calibration.center_samples.filter((s: any) => s.accepted);
      const rejected = calibration.center_samples.filter((s: any) => !s.accepted);
      const sharp = [...calibration.sharpness.coarse, ...calibration.sharpness.refined]
        .filter((s: any) => Number.isFinite(s.offset_m) && Number.isFinite(s.score));
      const lo = Math.min(...sharp.map((s: any) => s.offset_m));
      const hi = Math.max(...sharp.map((s: any) => s.offset_m));
      const max = Math.max(...sharp.map((s: any) => s.score), 1);
      const sharpPoint = (s: any) => [40 + (s.offset_m - lo) / (hi - lo || 1) * 820,
        330 - s.score / max * 290];
      const mapPoint = (key: string) => {
        const points = calibration.maps[key];
        const xs = points.map((p: number[]) => p[0]); const ys = points.map((p: number[]) => p[1]);
        const x = points[0][0]; const y = points[0][1];
        return [20 + (x - Math.min(...xs)) / (Math.max(...xs) - Math.min(...xs) || 1) * 560,
          340 - (y - Math.min(...ys)) / (Math.max(...ys) - Math.min(...ys) || 1) * 320];
      };
      return {
        acceptedColors: [colors.odom, colors.imu, colors.existing].map(color => hasColor('center-scatter', color)),
        rejectedColor: hasColor('center-scatter', colors.rejected),
        coarseColor: calibration.sharpness.coarse.filter((s: any) => Number.isFinite(s.score)).some((s: any) => near('sharpness', ...sharpPoint(s), colors.coarse)),
        refinedColor: calibration.sharpness.refined.filter((s: any) => Number.isFinite(s.score)).some((s: any) => near('sharpness', ...sharpPoint(s), colors.refined)),
        recordedMap: near('recorded-map', ...mapPoint('recorded'), [71, 85, 105]),
        estimatedMap: near('estimated-map', ...mapPoint('estimated'), [71, 85, 105]),
        canvases: ['center-scatter', 'sharpness', 'recorded-map', 'estimated-map'].map(id => {
          const canvas = document.getElementById(id) as HTMLCanvasElement;
          return Array.from(canvas.getContext('2d')!.getImageData(0, 0, canvas.width, canvas.height).data)
            .some((value, index) => index % 4 !== 3 && value !== 255 && value !== 0);
        }),
        _rgb: rgb('center-scatter', 0, 0),
      };
    }, { colors, calibration });
    expect(probes.acceptedColors).toEqual([true, true, true]);
    expect(probes.rejectedColor).toBe(true);
    expect(probes.coarseColor).toBe(true);
    expect(probes.refinedColor).toBe(true);
    expect(probes.recordedMap).toBe(true);
    expect(probes.estimatedMap).toBe(true);
    expect(probes.canvases).toEqual([true, true, true, true]);

    await page.goto(pathToFileURL(join(inconclusive, 'report.html')).href, { waitUntil: 'load' });
    await page.waitForTimeout(100);
    expect(consoleErrors).toEqual([]);
    expect(pageErrors).toEqual([]);
    await expect(page.locator('#estimated')).toHaveText('—');
  } finally {
    rmSync(root, { recursive: true, force: true });
  }
});
