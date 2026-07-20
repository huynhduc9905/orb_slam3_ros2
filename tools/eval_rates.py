import json
import sys
for i in range(2, 10):
  path = f'artifacts/tracking-performance-20260719_133536/orb-only-{i}x/tracking_benchmark.json'
  try:
    r = json.load(open(path))
    fps = r['tracking_fps']
    print(f'{i}x: fps={fps:.2f} target={i*30}')
  except Exception as e:
    pass
