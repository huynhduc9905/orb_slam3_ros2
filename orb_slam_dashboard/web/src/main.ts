/**
 * Placeholder entry for Vite build (UI lands in Task 3).
 * Task 2 only ships the transport layer (types / foxglove / store).
 */
export { ALLOWED_TOPICS, createDashboardConnection } from "./foxglove";
export { createDashboardStore } from "./store";
export type { DashboardState } from "./types";

const root = document.getElementById("root");
if (root) {
  root.textContent = "ORB Lidar Dashboard (transport ready)";
}
