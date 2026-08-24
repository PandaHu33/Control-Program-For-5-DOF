# Canonical real-time performance check — 2026-08-24

## Result

The optimized canonical producer sustained 50 Hz in a 10-second synthetic
control-path benchmark. The browser canvas sustained every supported display
rate; 50 FPS is the highest meaningful rate for a 50 Hz source, while the
production default remains 30 FPS.

## Backend benchmark

| Metric | Result | Acceptance |
|---|---:|---:|
| Published rate | 49.997 Hz | 49–51 Hz |
| Interval P95 | 20.40 ms | ≤22 ms |
| Interval P99 | 20.55 ms | ≤30 ms |
| Critical compute P95 | 1.11 ms | <20 ms |
| Recording enqueue P95 | 0.031 ms | non-blocking |
| Deadline misses | 0 | <0.5% |
| Canonical records | 500 enqueued / 500 written | zero loss |
| Recording queue peak | 1 / 4096 | no backlog |

The benchmark exercised canonical planning, semantic bundling, arm IK, atomic
command publication, asynchronous JSONL recording and performance telemetry.
The 500-row recording was about 4.7 MB and was drained before close.

## Browser benchmark

The local page received a synthetic canonical schema-4 stream at 50 Hz. Each
mode ran for approximately 7 seconds on the current workstation.

| Target | Receive rate | Render rate | Draw P95 | Data age P95 | Longest stall | Overwritten frames |
|---:|---:|---:|---:|---:|---:|---:|
| 20 FPS | 49.976 Hz | 20.000 FPS | 0.40 ms | 19.5 ms | 13.9 ms | 208 |
| 30 FPS | 50.005 Hz | 30.000 FPS | 0.40 ms | 19.4 ms | 13.8 ms | 139 |
| 40 FPS | 50.010 Hz | 39.999 FPS | 0.40 ms | 19.2 ms | 7.3 ms | 69 |
| 50 FPS | 49.997 Hz | 49.999 FPS | 0.40 ms | 13.0 ms | 7.5 ms | 0 |

Overwritten frames are intentional latest-only coalescing when the display rate
is below the receive rate. They do not represent lost control or recording
samples.

The supported-rate check identifies **50 FPS as the highest stable synthetic
rate on this workstation**. This is not an automatic production setting: the
viewer remains fixed at 30 FPS unless `canonical_render_fps` is explicitly set.

## Automated regression

- Main repository: `159 passed, 3 skipped`.
- WA100 child repository: `7 passed`; all three existing C++ test executables
  reported `PASS`.

## Remaining live-system gate

No bridge, camera, WA100, glove or controller service was listening during this
check. Before the next experiment, run keyboard, gamepad and VR-controller
conditions for at least five minutes each with both cameras and full recording
enabled. The live run must retain 49–51 Hz, interval P95 ≤22 ms, interval P99
≤30 ms, deadline misses <0.5%, zero recording loss and command age <100 ms.
The short synthetic runs above cannot replace this three-condition, three-minute
per-display-rate hardware acceptance test.
