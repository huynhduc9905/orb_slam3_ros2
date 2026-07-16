import { defineConfig } from "@playwright/test";
import path from "node:path";

export default defineConfig({
  testDir: "./e2e",
  testMatch: "direct-dashboard.spec.ts",
  timeout: 15_000,
  expect: { timeout: 5_000 },
  use: {
    baseURL: "http://127.0.0.1:51872",
    browserName: "chromium",
    channel: undefined,
    headless: true,
    screenshot: "only-on-failure",
    trace: "retain-on-failure",
    launchOptions: {
      executablePath: process.env.PLAYWRIGHT_CHROME_PATH || "/run/current-system/sw/bin/google-chrome",
    },
  },
  webServer: {
    command: `python3 -m http.server 51872 --bind 127.0.0.1 --directory ${path.resolve("../../orb_slam_bringup/web")}`,
    url: "http://127.0.0.1:51872/",
    reuseExistingServer: false,
  },
});
