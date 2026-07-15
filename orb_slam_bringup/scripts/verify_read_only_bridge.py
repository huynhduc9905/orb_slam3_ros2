#!/usr/bin/env python3
"""Live foxglove_bridge capability check (ENV-DEFERRED until package is installed).

Usage (after foxglove_bridge is on AMENT_PREFIX_PATH):

  # Terminal A — bind loopback for local probe
  ros2 launch orb_slam_bringup dashboard.launch.py \\
      dashboard_host:=127.0.0.1 http_port:=51871 websocket_port:=8765

  # Terminal B
  python3 orb_slam_bringup/scripts/verify_read_only_bridge.py \\
      --url ws://127.0.0.1:8765

Expected:
  - serverInfo.capabilities == []
  - client advertise / publish rejected or ignored
  - service call rejected or ignored
  - parameter request rejected or ignored
  - no new ROS graph writers appear (spot-check: ros2 topic list / ros2 node list)

If foxglove_bridge is not installed:
  ros2 pkg executables foxglove_bridge   # empty / package not found
  → skip this script; rely on test_read_only_bridge_locks_down_capabilities.
"""

from __future__ import annotations

import argparse
import json
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url",
        default="ws://127.0.0.1:8765",
        help="foxglove_bridge WebSocket URL",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="seconds to wait for serverInfo",
    )
    args = parser.parse_args()

    try:
        import websocket  # type: ignore  # websocket-client
    except ImportError:
        print(
            "websocket-client not installed; try: pip install websocket-client",
            file=sys.stderr,
        )
        return 2

    print(f"connecting to {args.url} ...")
    ws = websocket.create_connection(args.url, timeout=args.timeout)
    try:
        # Foxglove WebSocket protocol: first server message is serverInfo (JSON).
        deadline = time.time() + args.timeout
        server_info = None
        while time.time() < deadline:
            raw = ws.recv()
            if isinstance(raw, bytes):
                # Binary frames are not serverInfo; keep waiting for text.
                continue
            msg = json.loads(raw)
            if msg.get("op") == "serverInfo" or "capabilities" in msg:
                server_info = msg
                break
            # Some protocol versions nest under "op":"serverInfo"
            if isinstance(msg, dict) and msg.get("op") == "serverInfo":
                server_info = msg
                break

        if server_info is None:
            print("FAIL: no serverInfo received within timeout", file=sys.stderr)
            return 1

        print("serverInfo:", json.dumps(server_info, indent=2))
        caps = server_info.get("capabilities", None)
        if caps is None and isinstance(server_info.get("serverInfo"), dict):
            caps = server_info["serverInfo"].get("capabilities")

        if caps != []:
            print(f"FAIL: expected capabilities == [], got {caps!r}", file=sys.stderr)
            return 1
        print("OK: capabilities == []")

        # Attempt client advertise (should be rejected/ignored — no clientPublish).
        advertise = {
            "op": "advertise",
            "channelId": 1,
            "topic": "/verify_read_only_should_fail",
            "encoding": "json",
            "schemaName": "std_msgs/msg/String",
        }
        ws.send(json.dumps(advertise))
        print("sent advertise (expect reject/ignore)")

        # Attempt service call (should fail — no services capability).
        call = {
            "op": "callService",
            "callId": 1,
            "serviceId": 1,
            "encoding": "json",
            "args": {},
        }
        ws.send(json.dumps(call))
        print("sent callService (expect reject/ignore)")

        # Attempt getParameters (should fail — no parameters capability).
        get_params = {
            "op": "getParameters",
            "id": "verify-params",
            "parameterNames": [],
        }
        ws.send(json.dumps(get_params))
        print("sent getParameters (expect reject/ignore)")

        # Drain any status/error replies briefly.
        end = time.time() + 1.0
        while time.time() < end:
            try:
                ws.settimeout(0.2)
                raw = ws.recv()
                if isinstance(raw, str):
                    print("reply:", raw[:500])
            except Exception:
                break

        print(
            "OK: probe finished. Manually confirm no ROS writers:\n"
            "  ros2 node list | grep -i foxglove\n"
            "  ros2 topic list | grep verify_read_only"
        )
        return 0
    finally:
        ws.close()


if __name__ == "__main__":
    sys.exit(main())
