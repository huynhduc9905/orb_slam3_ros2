import subprocess
try:
  res = subprocess.check_output(['python3', '-m', 'orb_slam_bringup.tracking_benchmark', 'select', '--source-camera-hz', '15', '--result', 'artifacts/tracking-performance-20260719_134544/orb-only-2x/tracking_benchmark.json', '--result', 'artifacts/tracking-performance-20260719_134544/orb-only-3x/tracking_benchmark.json', '--result', 'artifacts/tracking-performance-20260719_134544/orb-only-4x/tracking_benchmark.json'], stderr=subprocess.STDOUT)
  print('stdout:', res.decode('utf-8'))
except subprocess.CalledProcessError as e:
  print('error output:', e.output.decode('utf-8'))
