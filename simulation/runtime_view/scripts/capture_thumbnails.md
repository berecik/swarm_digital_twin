# Mission Thumbnail Capture (Phase 7-2)

The mission cards in `web/index.html` reference PNGs in
`simulation/runtime_view/web/img/`. Today those are Pillow-generated
placeholders (`fw.png`, `physics.png`, `real.png`, `single.png`,
`swarm3.png`, `swarm6.png`). The Phase 7-2 acceptance criterion asks for
real screenshots from SITL runs; this is an **operator-only** workflow
because CI cannot drive a browser to screenshot the live 3D view.

## Per-mission steps

For each mission listed in `missions.json`:

1. Start the live view with that mission:
   ```bash
   ./run_scenario.sh --single-live          # for `single`
   ./run_scenario.sh --physics-live --loop  # for `physics`
   ./run_scenario.sh --replay-live <path>   # for `real-log` (use a real .npz/.BIN)
   ./run_scenario.sh --swarm 3              # for `swarm-3`
   ./run_scenario.sh --swarm 6              # for `swarm-6`
   ```
2. Wait for the drone(s) to be mid-mission (15–30 s in usually works).
3. Open `http://127.0.0.1:8765/live` in a browser, frame the scene with
   waypoints + drone(s) visible.
4. Take a 640 × 360 screenshot (Cmd-Shift-4 on macOS, then resize) and
   save as `simulation/runtime_view/web/img/<id>.png` matching the
   mission `id` in `missions.json`.
5. Commit the new PNG.

## Acceptance check

After replacing each placeholder, verify:

- All 6 mission cards render real screenshots in the launcher.
- Each thumbnail shows the drone(s) in flight with terrain and
  waypoints visible (so the viewer state matches what the user gets
  when they click `LAUNCH`).
- The "drones-always-visible invariant" (Section 0 of
  `todo/live_view_backlog.md`) holds — i.e. the placeholder mesh was
  never blank during the capture.

## Why not automated?

The natural automation path (Playwright + headless Chrome) would add
a Node toolchain and ~200 MB of browser deps to CI for one acceptance
item. The cost-benefit doesn't pencil today; revisit when Phase 7's
broader frontend testing story (DOM smoke, replay-loop regression)
needs Playwright anyway.
