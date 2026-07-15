#!/usr/bin/env python3
"""Static file server for the installed ORB lidar dashboard.

Read-only: serves share/orb_slam_dashboard/web. No ROS publishers, services,
or parameter writes.
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


log = logging.getLogger("orb_slam_dashboard_server")


def resolve_web_root() -> Path:
    """Locate installed web assets or a local web/dist for development."""
    # 1) AMENT_PREFIX_PATH share install
    prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    for entry in prefix.split(os.pathsep):
        if not entry:
            continue
        candidate = Path(entry) / "share" / "orb_slam_dashboard" / "web"
        if (candidate / "index.html").is_file():
            return candidate

    # 2) Relative to this file when run from source tree after npm build
    here = Path(__file__).resolve()
    # orb_slam_dashboard/orb_slam_dashboard/server.py → ../../web/dist
    src_dist = here.parents[1] / "web" / "dist"
    if (src_dist / "index.html").is_file():
        return src_dist

    # 3) COLCON_PREFIX_PATH fallback
    colcon = os.environ.get("COLCON_PREFIX_PATH", "")
    for entry in colcon.split(os.pathsep):
        if not entry:
            continue
        candidate = Path(entry) / "share" / "orb_slam_dashboard" / "web"
        if (candidate / "index.html").is_file():
            return candidate

    raise SystemExit(
        "orb_slam_dashboard_server: could not find share/orb_slam_dashboard/web "
        "(install the package or run npm run build in web/)."
    )


class DashboardHandler(SimpleHTTPRequestHandler):
    """Serve static dashboard files; no-store for index.html."""

    def __init__(self, *args, directory: str | None = None, **kwargs):
        super().__init__(*args, directory=directory, **kwargs)

    def end_headers(self) -> None:
        path = self.path.split("?", 1)[0].split("#", 1)[0]
        if path in ("/", "/index.html") or path.endswith("/index.html"):
            self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt: str, *args) -> None:
        log.info("%s - %s", self.address_string(), fmt % args)


def lan_urls(host: str, port: int) -> list[str]:
    """Guess reachable LAN / Tailscale URLs for operator convenience."""
    urls: list[str] = []
    if host in ("0.0.0.0", "::", ""):
        urls.append(f"http://127.0.0.1:{port}/")
        try:
            hostname = socket.gethostname()
            for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
                addr = info[4][0]
                if addr.startswith("127."):
                    continue
                urls.append(f"http://{addr}:{port}/")
        except OSError:
            pass
        # Tailscale often exposes 100.x.y.z
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("1.1.1.1", 80))
                local = s.getsockname()[0]
                if not local.startswith("127."):
                    u = f"http://{local}:{port}/"
                    if u not in urls:
                        urls.append(u)
        except OSError:
            pass
    else:
        urls.append(f"http://{host}:{port}/")
    # de-dupe preserve order
    seen: set[str] = set()
    out: list[str] = []
    for u in urls:
        if u not in seen:
            seen.add(u)
            out.append(u)
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Serve the read-only ORB lidar map dashboard (static files only)."
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Bind address (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="Bind port (default: 8080)",
    )
    parser.add_argument(
        "--web-root",
        default=None,
        help="Override path to built web assets (defaults to install share path)",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="[%(name)s] %(levelname)s: %(message)s",
    )

    web_root = Path(args.web_root) if args.web_root else resolve_web_root()
    if not (web_root / "index.html").is_file():
        log.error("index.html not found under %s", web_root)
        return 1

    handler = partial(DashboardHandler, directory=str(web_root))
    server = ThreadingHTTPServer((args.host, args.port), handler)

    urls = lan_urls(args.host, args.port)
    log.info("Serving %s", web_root)
    for u in urls:
        log.info("Dashboard URL: %s", u)
    if not urls:
        log.info("Dashboard listening on %s:%s", args.host, args.port)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Shutting down")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
