import sys
import json
from orb_slam_bringup.tracking_benchmark import select_stress_point, load_result

try:
    results = [load_result(f"artifacts/tracking-performance-20260719_134544/orb-only-{i}x/tracking_benchmark.json") for i in range(2, 5)]
    res = select_stress_point(results, 15.0)
    print("SUCCESS")
    print(json.dumps(res.as_dict()))
except Exception as e:
    print(f"FAILED: {e}")
