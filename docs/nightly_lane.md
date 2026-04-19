# Nightly K8s + Gazebo Validation Lane

Some Phase 1–6 audit-fix items can only be honored against a running
Kubernetes cluster + Gazebo runtime. CI today is Python-only (pytest +
shell + JS syntax) — fast, hermetic, and free, but unable to start a
StatefulSet, inject pod restarts, or query a Gazebo heightmap. This
document is the contract for the nightly lane that will eventually run
those items.

## Scope (deferred audit items)

| Phase | Item | Why nightly |
|:---|:---|:---|
| 3 | Live Gazebo terrain-parity (`get_elevation` vs `gz::physics::HeightmapShape::HeightAt`) | Needs Gazebo container + ROS topic |
| 5 | Hard wind stress envelope (mission-completion + attitude-error gates per profile class) | Needs PX4-tuned controller; default Python PD controller produces 100°+ transient roll, would fail every gate |
| 5 | Gazebo wind-plugin parity (`libgazebo_wind_plugin.so` numbers vs `WindField.get_wind_velocity()`) | Needs Gazebo container |
| 6 | K8s fault injection (`pod_restart`, `packet_loss`, `telemetry_delay`, `sensor_dropout`) — measure detect/respond/recover timestamps | Needs K8s + `kubectl delete pod`, `tc qdisc`, etc. |
| 6 | Scalability gates (pod startup time, scheduling delay) for 1/3/6/12-drone swarms | Needs K8s + Helm |
| 7 | Headless DOM smoke + replay-loop snapshot for the live view | Needs Playwright/headless Chrome |

## Invocation contract

When the nightly lane lands it MUST expose a single entry point:

```bash
./run_scenario.sh --nightly-lane [--subset terrain|wind|matrix|view|all]
```

That entry point should:

1. Spin up the required infrastructure (`helm install` for K8s; `gz sim`
   for Gazebo; Playwright for the viewer subset).
2. Run the deferred checks in their natural order.
3. Write report artefacts into `reports/nightly/<timestamp>/`:
   - `terrain_parity.json` — per-sample `|Δz|` between `terrain.py` and
     Gazebo `HeightAt`. Pass: max < 0.5 m.
   - `wind_envelope.json` — per-profile mission completion +
     max-attitude. Pass: thresholds in
     `acceptance_report.WIND_ATTITUDE_LIMITS_DEG_K8S` and
     `WIND_COMPLETION_TARGETS`.
   - `wind_plugin_parity.json` — Gazebo wind-plugin samples vs
     `WindField` reference. Pass: |Δv| < 0.2 m/s.
   - `fault_injection.json` — per-fault timestamps for inject /
     detect / respond / recover. Pass: `recover - inject < 30 s`.
   - `scalability.json` — pod-startup + scheduling-delay per
     swarm-size. Pass: thresholds in `todo/k8s_test_matrix.md`.
   - `view_smoke.json` — Playwright DOM snapshots before telemetry,
     after first sample, mid-replay-loop. Pass: drone meshes
     present in every snapshot.
4. Return non-zero exit if any subset fails.

## What CI keeps doing

The Python-only CI lane stays the gate for everything that doesn't
require infrastructure: scenario matrix Python pipeline, terrain/wind
manifest loaders, mesh export-roundtrip parity, KPI schema, AGL
detection in the simulation loop, attitude KPI export (without
gating), live-view static invariant checks. The matrix runner
already keeps the strict K8s thresholds as `*_K8S` constants
(`acceptance_report.MIN_SEPARATION_M_K8S`,
`NEAR_MISS_PER_MISSION_K8S`, `TRAJECTORY_RMSE_*_K8S`) so the nightly
lane can adopt them without re-deriving.

## Owners

The nightly lane is owned by whoever runs the Kubernetes cluster the
project ships against. Until that operator exists, the items in the
table above stay deferred — not pending — and the gates remain
documented in code as `*_K8S` constants.

## Status today

Not yet implemented. Tracked in
[`ROADMAP.md`](../ROADMAP.md#phase-3-audit-fixes-2026-04-19--deferred-to-nightly-lane)
and [`TESTING.md`](../TESTING.md).
