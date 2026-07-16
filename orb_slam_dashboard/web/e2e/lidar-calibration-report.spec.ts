import { expect, test } from '@playwright/test';

const reportFixture = `<!doctype html><html><head><meta charset="utf-8"><style>
body{margin:0;padding:16px;font:14px system-ui;background:#f5f7fa}main{max-width:1180px;margin:auto}section{background:white;border:1px solid #ccd5e0;border-radius:10px;margin:10px 0;padding:12px}canvas{display:block;width:100%;height:auto;max-width:900px;border:1px solid #ccd5e0}.maps{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.map-card{min-width:0}table{width:100%;border-collapse:collapse}td,th{padding:6px;border-bottom:1px solid #e5e7eb;text-align:left}@media(max-width:600px){.maps{grid-template-columns:1fr}body{padding:6px}section{padding:8px}table{font-size:11px;display:block;overflow-x:auto;white-space:nowrap}}
</style></head><body><main><h1>Lidar rotation-center calibration</h1><section><h2>LIKELY_OFFSET_ERROR</h2><p>Recorded offset: 0.260 m; estimated offset: 0.245 m</p></section><section><h2>Raw center scatter</h2><canvas id="scatter" width="900" height="360"></canvas></section><section><h2>Method estimates</h2><table><tr><th>Method</th><th>Center</th><th>Delta</th><th>95% CI</th><th>Accepted/attempted</th></tr><tr><td>Odom</td><td>0.245, 0.001</td><td>-0.015</td><td>[0.240,0.250]</td><td>48/50</td></tr><tr><td>IMU</td><td>0.245, 0.001</td><td>-0.015</td><td>[0.240,0.250]</td><td>48/50</td></tr><tr><td>Existing /scan</td><td>0.245, 0.001</td><td>-0.015</td><td>[0.240,0.250]</td><td>48/50</td></tr></table></section><section><h2>Sharpness curve</h2><canvas id="sharpness" width="900" height="360"></canvas></section><section><h2>Map views</h2><div class="maps"><div class="map-card"><h3>Recorded map view</h3><canvas id="recorded" width="600" height="360"></canvas></div><div class="map-card"><h3>Estimated map view</h3><canvas id="estimated" width="600" height="360"></canvas></div></div></section></main><script>
for(const id of ['scatter','sharpness','recorded','estimated']){const c=document.getElementById(id),x=c.getContext('2d');x.fillStyle='#2563eb';x.fillRect(20,20,c.width-40,c.height-40);x.strokeStyle='#dc2626';x.beginPath();x.moveTo(20,c.height-20);x.lineTo(c.width-20,20);x.stroke()}
</script></body></html>`;

test('calibration report renders at every required viewport without runtime or network faults', async ({ page }) => {
  const consoleErrors: string[] = [];
  const requests: string[] = [];
  page.on('console', message => { if (message.type() === 'error') consoleErrors.push(message.text()); });
  page.on('request', request => requests.push(request.url()));
  await page.setContent(reportFixture, { waitUntil: 'load' });
  const requestsAfterLoad = requests.length;
  await page.waitForTimeout(100);
  expect(consoleErrors).toEqual([]);
  expect(requests.length).toBe(requestsAfterLoad);
  for (const id of ['scatter', 'sharpness', 'recorded', 'estimated']) {
    const canvas = page.locator(`#${id}`);
    await expect(canvas).toBeVisible();
    const nonblank = await canvas.evaluate((element) => {
      const context = (element as HTMLCanvasElement).getContext('2d');
      if (!context) return false;
      const pixels = context.getImageData(0, 0, (element as HTMLCanvasElement).width, (element as HTMLCanvasElement).height).data;
      return pixels.some(value => value !== 0);
    });
    expect(nonblank).toBe(true);
  }
  await expect(page.getByText('Raw center scatter')).toBeVisible();
  await expect(page.getByText('Existing /scan')).toBeVisible();
  await expect(page.getByText('Recorded map view')).toBeVisible();
  await expect(page.getByText('Estimated map view')).toBeVisible();
  const dimensions = await page.evaluate(() => ({ width: document.documentElement.scrollWidth, viewport: window.innerWidth }));
  expect(dimensions.width).toBeLessThanOrEqual(dimensions.viewport + 1);
});
