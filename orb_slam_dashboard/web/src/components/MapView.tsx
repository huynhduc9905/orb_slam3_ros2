import { useCallback, useEffect, useRef, useState } from "react";
import { Application, Container, Graphics, Sprite, Texture } from "pixi.js";
import { Eye, EyeOff, Maximize, WifiOff, ZoomIn, ZoomOut } from "lucide-react";

import type { DashboardState, OccupancyGrid } from "../types";
import {
  isAllUnknown,
  mapWorldBounds,
  occupancyToRGBA,
} from "../render/occupancy";
import {
  loopEdgesToSegments,
  pathToVertices,
  PATH_COLORS,
} from "../render/paths";
import { scanToPoints, SCAN_COLOR } from "../render/scan";

export interface LayerVisibility {
  orbPath: boolean;
  wheelPath: boolean;
  provisionalScan: boolean;
  loopEdges: boolean;
  keyframes: boolean;
}

const DEFAULT_LAYERS: LayerVisibility = {
  orbPath: true,
  wheelPath: true,
  provisionalScan: true,
  loopEdges: true,
  keyframes: true,
};

function hexNum(hex: string): number {
  return parseInt(hex.replace("#", ""), 16);
}

function canUsePixi(): boolean {
  if (typeof window === "undefined" || typeof document === "undefined") return false;
  // jsdom navigator.userAgent contains "jsdom" — skip Pixi (no real WebGL).
  if (typeof navigator !== "undefined" && /jsdom/i.test(navigator.userAgent)) {
    return false;
  }
  try {
    const c = document.createElement("canvas");
    return Boolean(c.getContext && (c.getContext("webgl") || c.getContext("webgl2")));
  } catch {
    return false;
  }
}

function LayerToggleButton({
  label,
  pressed,
  onToggle,
}: {
  label: string;
  pressed: boolean;
  onToggle: () => void;
}) {
  const Icon = pressed ? Eye : EyeOff;
  return (
    <button
      type="button"
      aria-label={label}
      title={label}
      aria-pressed={pressed}
      onClick={onToggle}
    >
      <Icon size={14} aria-hidden />
      {label}
    </button>
  );
}

export interface MapViewProps {
  state: DashboardState;
}

/**
 * Dominant PixiJS map surface.
 * Occupancy texture rebuilds only when mapRevision changes.
 * Path/scan geometry live on separate Graphics layers.
 */
export function MapView({ state }: MapViewProps) {
  const hostRef = useRef<HTMLDivElement | null>(null);
  const appRef = useRef<Application | null>(null);
  const worldRef = useRef<Container | null>(null);
  const mapSpriteRef = useRef<Sprite | null>(null);
  const orbGfxRef = useRef<Graphics | null>(null);
  const wheelGfxRef = useRef<Graphics | null>(null);
  const scanGfxRef = useRef<Graphics | null>(null);
  const loopGfxRef = useRef<Graphics | null>(null);
  const kfGfxRef = useRef<Graphics | null>(null);
  const robotGfxRef = useRef<Graphics | null>(null);

  const lastRenderedMap = useRef<OccupancyGrid | null>(null);
  const layersRef = useRef<LayerVisibility>({ ...DEFAULT_LAYERS });
  const [layers, setLayers] = useState<LayerVisibility>({ ...DEFAULT_LAYERS });
  const dragRef = useRef<{ x: number; y: number } | null>(null);
  const readyRef = useRef(false);

  layersRef.current = layers;

  const waiting =
    !state.map || isAllUnknown(state.map) || state.map.info.width === 0;

  const fitMap = useCallback(() => {
    const app = appRef.current;
    const world = worldRef.current;
    if (!app || !world || !state.map || state.map.info.width <= 0) return;
    const bounds = mapWorldBounds(state.map);
    const viewW = app.screen.width;
    const viewH = app.screen.height;
    if (viewW <= 0 || viewH <= 0 || bounds.width <= 0 || bounds.height <= 0) return;
    const pad = 0.9;
    const scale = Math.min(viewW / bounds.width, viewH / bounds.height) * pad;
    world.scale.set(scale, -scale); // flip Y: ROS y-up → screen y-down
    const cx = (bounds.minX + bounds.maxX) / 2;
    const cy = (bounds.minY + bounds.maxY) / 2;
    world.position.set(viewW / 2 - cx * scale, viewH / 2 + cy * scale);
  }, [state.map]);

  const zoomBy = useCallback((factor: number, cx?: number, cy?: number) => {
    const app = appRef.current;
    const world = worldRef.current;
    if (!app || !world) return;
    const ox = cx ?? app.screen.width / 2;
    const oy = cy ?? app.screen.height / 2;
    const beforeX = (ox - world.position.x) / world.scale.x;
    const beforeY = (oy - world.position.y) / world.scale.y;
    world.scale.x *= factor;
    world.scale.y *= factor;
    world.position.x = ox - beforeX * world.scale.x;
    world.position.y = oy - beforeY * world.scale.y;
  }, []);

  // Init Pixi application once (skipped in jsdom — pure render fns are unit-tested)
  useEffect(() => {
    const host = hostRef.current;
    if (!host || !canUsePixi()) return;
    let cancelled = false;
    const app = new Application();

    (async () => {
      try {
        await app.init({
          resizeTo: host,
          background: 0x171c21,
          antialias: true,
          autoDensity: true,
          resolution: typeof window !== "undefined" ? window.devicePixelRatio || 1 : 1,
          preference: "webgl",
        });
      } catch {
        // No WebGL / canvas: leave host empty; unit tests cover pure render fns
        return;
      }
      if (cancelled) {
        app.destroy(true);
        return;
      }
      host.appendChild(app.canvas);
      const world = new Container();
      const mapSprite = new Sprite();
      const orbGfx = new Graphics();
      const wheelGfx = new Graphics();
      const scanGfx = new Graphics();
      const loopGfx = new Graphics();
      const kfGfx = new Graphics();
      const robotGfx = new Graphics();
      world.addChild(mapSprite);
      world.addChild(loopGfx);
      world.addChild(wheelGfx);
      world.addChild(orbGfx);
      world.addChild(scanGfx);
      world.addChild(kfGfx);
      world.addChild(robotGfx);
      app.stage.addChild(world);

      appRef.current = app;
      worldRef.current = world;
      mapSpriteRef.current = mapSprite;
      orbGfxRef.current = orbGfx;
      wheelGfxRef.current = wheelGfx;
      scanGfxRef.current = scanGfx;
      loopGfxRef.current = loopGfx;
      kfGfxRef.current = kfGfx;
      robotGfxRef.current = robotGfx;
      readyRef.current = true;

      const onPointerDown = (e: PointerEvent) => {
        dragRef.current = { x: e.clientX, y: e.clientY };
        host.setPointerCapture?.(e.pointerId);
      };
      const onPointerMove = (e: PointerEvent) => {
        if (!dragRef.current || !worldRef.current) return;
        const dx = e.clientX - dragRef.current.x;
        const dy = e.clientY - dragRef.current.y;
        dragRef.current = { x: e.clientX, y: e.clientY };
        worldRef.current.position.x += dx;
        worldRef.current.position.y += dy;
      };
      const onPointerUp = (e: PointerEvent) => {
        dragRef.current = null;
        try {
          host.releasePointerCapture?.(e.pointerId);
        } catch {
          /* ignore */
        }
      };
      const onWheel = (e: WheelEvent) => {
        e.preventDefault();
        const rect = host.getBoundingClientRect();
        const factor = e.deltaY > 0 ? 0.9 : 1.1;
        zoomBy(factor, e.clientX - rect.left, e.clientY - rect.top);
      };
      host.addEventListener("pointerdown", onPointerDown);
      host.addEventListener("pointermove", onPointerMove);
      host.addEventListener("pointerup", onPointerUp);
      host.addEventListener("pointercancel", onPointerUp);
      host.addEventListener("wheel", onWheel, { passive: false });

      // store cleanup on host dataset via closure
      (host as HTMLDivElement & { __mapCleanup?: () => void }).__mapCleanup = () => {
        host.removeEventListener("pointerdown", onPointerDown);
        host.removeEventListener("pointermove", onPointerMove);
        host.removeEventListener("pointerup", onPointerUp);
        host.removeEventListener("pointercancel", onPointerUp);
        host.removeEventListener("wheel", onWheel);
      };
    })();

    return () => {
      cancelled = true;
      readyRef.current = false;
      const cleanup = (host as HTMLDivElement & { __mapCleanup?: () => void }).__mapCleanup;
      cleanup?.();
      if (appRef.current) {
        try {
          appRef.current.destroy(true, { children: true, texture: true });
        } catch {
          /* ignore */
        }
      }
      appRef.current = null;
      worldRef.current = null;
    };
  }, [zoomBy]);

  // Rebuild occupancy texture whenever a new grid arrives.
  // Keyed on the grid object identity (the store replaces `state.map` only when
  // a fresh OccupancyGrid is received), NOT on mapRevision. The grid and the
  // revision counter travel on separate topics, so gating on the counter could
  // drop the final corrected grid if it landed after its revision was already
  // consumed — leaving a stale (pre-loop-closure) texture on screen.
  useEffect(() => {
    if (!readyRef.current || !mapSpriteRef.current) return;
    if (!state.map || state.map.info.width <= 0) return;
    // Same grid object already rasterized — nothing new to draw. Path/scan/robot
    // layer updates keep the same `state.map` reference, so they no-op here.
    if (lastRenderedMap.current === state.map) {
      if (mapSpriteRef.current.texture && mapSpriteRef.current.texture !== Texture.EMPTY) {
        return;
      }
    }
    lastRenderedMap.current = state.map;
    const { width, height, data } = occupancyToRGBA(state.map);
    if (width <= 0 || height <= 0) return;

    // Pixi Texture from raw RGBA
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const img = ctx.createImageData(width, height);
    // Flip vertically so ROS row0 (y=0 bottom) appears at bottom of sprite
    for (let y = 0; y < height; y++) {
      const srcRow = y * width * 4;
      const dstRow = (height - 1 - y) * width * 4;
      img.data.set(data.subarray(srcRow, srcRow + width * 4), dstRow);
    }
    ctx.putImageData(img, 0, 0);
    const tex = Texture.from(canvas);
    const sprite = mapSpriteRef.current;
    const old = sprite.texture;
    sprite.texture = tex;
    if (old && old !== Texture.EMPTY) {
      try {
        old.destroy(true);
      } catch {
        /* ignore */
      }
    }
    const bounds = mapWorldBounds(state.map);
    sprite.position.set(bounds.minX, bounds.minY);
    sprite.width = bounds.width;
    sprite.height = bounds.height;
    // After flip in image, sprite local y grows up in world when world.scale.y is negative
    // With world.scale.y = -s, sprite at minY with height positive works with texture flip.
    fitMap();
  }, [state.map, fitMap]);

  // Update path / scan / edge / robot graphics independently
  useEffect(() => {
    if (!readyRef.current) return;
    const vis = layersRef.current;

    const drawPath = (gfx: Graphics | null, verts: Float32Array, color: number, visible: boolean) => {
      if (!gfx) return;
      gfx.clear();
      gfx.visible = visible;
      if (!visible || verts.length < 4) return;
      gfx.moveTo(verts[0]!, verts[1]!);
      for (let i = 2; i < verts.length; i += 2) {
        gfx.lineTo(verts[i]!, verts[i + 1]!);
      }
      gfx.stroke({ width: 0.05, color, pixelLine: false });
    };

    drawPath(
      orbGfxRef.current,
      pathToVertices(state.orbPath),
      hexNum(PATH_COLORS.orb),
      vis.orbPath,
    );
    drawPath(
      wheelGfxRef.current,
      pathToVertices(state.wheelPath),
      hexNum(PATH_COLORS.wheel),
      vis.wheelPath,
    );

    const scanGfx = scanGfxRef.current;
    if (scanGfx) {
      scanGfx.clear();
      scanGfx.visible = vis.provisionalScan;
      if (vis.provisionalScan) {
        const pts = scanToPoints(state.provisionalScan);
        for (let i = 0; i < pts.length; i += 2) {
          scanGfx.circle(pts[i]!, pts[i + 1]!, 0.04);
        }
        scanGfx.fill({ color: hexNum(SCAN_COLOR) });
      }
    }

    const loopGfx = loopGfxRef.current;
    if (loopGfx) {
      loopGfx.clear();
      loopGfx.visible = vis.loopEdges;
      if (vis.loopEdges) {
        const segs = loopEdgesToSegments(state.loopEdges);
        for (let i = 0; i + 3 < segs.length; i += 4) {
          loopGfx.moveTo(segs[i]!, segs[i + 1]!);
          loopGfx.lineTo(segs[i + 2]!, segs[i + 3]!);
        }
        loopGfx.stroke({ width: 0.04, color: hexNum(PATH_COLORS.loopEdge) });
      }
    }

    const kfGfx = kfGfxRef.current;
    if (kfGfx) {
      kfGfx.clear();
      kfGfx.visible = vis.keyframes;
      if (vis.keyframes) {
        for (const p of state.keyframes) {
          kfGfx.circle(p.x, p.y, 0.06);
        }
        kfGfx.fill({ color: 0x94a3b8 });
      }
    }

    const robotGfx = robotGfxRef.current;
    if (robotGfx) {
      robotGfx.clear();
      const pose = state.tracking.pose;
      if (pose && state.tracking.poseValid) {
        const r = 0.12;
        robotGfx.circle(pose.x, pose.y, r);
        robotGfx.fill({ color: hexNum(PATH_COLORS.robotOk) });
        robotGfx.moveTo(pose.x, pose.y);
        robotGfx.lineTo(
          pose.x + Math.cos(pose.yaw) * r * 2,
          pose.y + Math.sin(pose.yaw) * r * 2,
        );
        robotGfx.stroke({ width: 0.04, color: 0xffffff });
      }
    }
  }, [
    state.orbPath,
    state.wheelPath,
    state.provisionalScan,
    state.loopEdges,
    state.keyframes,
    state.tracking,
    layers,
  ]);

  function toggle(key: keyof LayerVisibility) {
    setLayers((prev) => ({ ...prev, [key]: !prev[key] }));
  }

  return (
    <div className="map-view" data-testid="map-view">
      <div className="map-canvas-host" ref={hostRef} />
      <div className="layer-toggles">
        <LayerToggleButton
          label="ORB path"
          pressed={layers.orbPath}
          onToggle={() => toggle("orbPath")}
        />
        <LayerToggleButton
          label="Wheel path"
          pressed={layers.wheelPath}
          onToggle={() => toggle("wheelPath")}
        />
        <LayerToggleButton
          label="Provisional scan"
          pressed={layers.provisionalScan}
          onToggle={() => toggle("provisionalScan")}
        />
        <LayerToggleButton
          label="Loop edges"
          pressed={layers.loopEdges}
          onToggle={() => toggle("loopEdges")}
        />
        <LayerToggleButton
          label="Keyframes"
          pressed={layers.keyframes}
          onToggle={() => toggle("keyframes")}
        />
      </div>
      <div className="map-toolbar">
        <button
          type="button"
          aria-label="Zoom in"
          title="Zoom in"
          onClick={() => zoomBy(1.2)}
        >
          <ZoomIn size={16} aria-hidden />
        </button>
        <button
          type="button"
          aria-label="Zoom out"
          title="Zoom out"
          onClick={() => zoomBy(1 / 1.2)}
        >
          <ZoomOut size={16} aria-hidden />
        </button>
        <button type="button" aria-label="Fit map" title="Fit map" onClick={fitMap}>
          <Maximize size={16} aria-hidden />
        </button>
      </div>
      {waiting ? (
        <div className="map-waiting" data-testid="map-waiting">
          Waiting for committed map
        </div>
      ) : null}
      {state.connection === "disconnected" || state.connection === "error" ? (
        <div className="connection-overlay" data-testid="connection-overlay">
          <WifiOff size={36} role="img" aria-label="Wifi off" />
          <span>
            {state.connection === "error" ? "Connection error" : "Disconnected"}
          </span>
        </div>
      ) : null}
    </div>
  );
}
