import { defineConfig, devices } from "@playwright/test";
import path from "node:path";
import { fileURLToPath } from "node:url";

/**
 * Use system Chrome on NixOS — `npx playwright install` downloads
 * dynamically-linked browsers that will not run here.
 *
 * Spec lives at ../test/dashboard.spec.ts (brief path) and is linked
 * from e2e/ so Node resolves @playwright/test from this package.
 */
const chromePath =
  process.env.PLAYWRIGHT_CHROME_PATH ??
  "/run/current-system/sw/bin/google-chrome";

const configDir = path.dirname(fileURLToPath(import.meta.url));

export default defineConfig({
  testDir: path.join(configDir, "e2e"),
  testMatch: "**/dashboard.spec.ts",
  fullyParallel: false,
  forbidOnly: !!process.env.CI,
  retries: 0,
  workers: 1,
  reporter: "list",
  use: {
    baseURL: "http://127.0.0.1:4173",
    trace: "off",
    screenshot: "only-on-failure",
    launchOptions: {
      executablePath: chromePath,
      args: ["--no-sandbox", "--disable-gpu", "--disable-dev-shm-usage"],
    },
  },
  webServer: {
    command: "npm run preview -- --host 127.0.0.1 --port 4173",
    cwd: configDir,
    url: "http://127.0.0.1:4173",
    reuseExistingServer: !process.env.CI,
    timeout: 120_000,
  },
  projects: [
    {
      name: "chrome",
      use: {
        ...devices["Desktop Chrome"],
        channel: undefined,
        launchOptions: {
          executablePath: chromePath,
          args: ["--no-sandbox", "--disable-gpu", "--disable-dev-shm-usage"],
        },
      },
    },
  ],
});
