import { createRoot } from "react-dom/client";

import { App } from "./App";
import { createDashboardStore } from "./store";
import type { DashboardStore } from "./store";
import "./styles.css";

const rootEl = document.getElementById("root");
if (!rootEl) {
  throw new Error("#root element missing");
}

const store = createDashboardStore();

// Playwright / manual fixtures: expose store for injectFixture.
const w = window as unknown as {
  __dashboardStore?: DashboardStore;
};
w.__dashboardStore = store;

const params = new URLSearchParams(window.location.search);
const connectOnMount = params.get("fixture") !== "1";

createRoot(rootEl).render(
  <App store={store} connectOnMount={connectOnMount} />,
);
