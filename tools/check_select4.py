import sys
from orb_slam_bringup.tracking_benchmark import main
sys.exit(main(['select', '--source-camera-hz', '15', '--result', 'artifacts/tracking-performance-20260719_134544/orb-only-3x/tracking_benchmark.json', '--result', 'artifacts/tracking-performance-20260719_134544/orb-only-4x/tracking_benchmark.json']))
