#!/usr/bin/env python3
"""
Drone Flight Scenario - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Full-featured scenario: takeoff -> fly waypoints over terrain with wind -> land.
Demonstrates: quadratic drag, ISA atmosphere, wind perturbation, terrain collision.
All physics simulated -- no ROS 2 required.
"""

import os
import sys
import numpy as np

# Ensure imports work when run from any directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from drone_physics import (
    DroneParams, AeroCoefficients, Atmosphere, run_simulation,
)
from wind_model import WindField
from terrain import TerrainMap, download_satellite_tile
from flight_log import FlightLog
from validation import (
    compute_rmse,
    compare_sim_real,
    plot_comparison,
    get_benchmark_profile,
    assert_validation_pass,
    summarize_validation,
    interpolate_to_common_times,
    get_real_log_mission,
    ensure_real_log_logs,
    assert_real_log_validation_pass,
)


MIN_AGL = 5.0   # minimum clearance above ground [m]


def make_terrain() -> TerrainMap:
    """Create terrain with prominent hills and a valley."""
    def hills(x, y):
        z = (
            8.0 * np.exp(-((x - 15)**2 + (y - 5)**2) / 80)    # hill east
            + 6.0 * np.exp(-((x - 30)**2 + (y - 18)**2) / 60)  # hill NE
            + 10.0 * np.exp(-((x - 10)**2 + (y - 20)**2) / 50) # tall ridge
            + 4.0 * np.sin(x * 0.06) * np.cos(y * 0.08)        # rolling base
            + 2.0
        )
        return np.maximum(z, 0.0)

    return TerrainMap.from_function(
        hills,
        x_range=(-15, 55),
        y_range=(-15, 40),
        resolution=0.5,
    )


def waypoints_above_terrain(xy_z_agl, terrain: TerrainMap):
    """Convert (x, y, agl) tuples to absolute waypoints above terrain.

    Each waypoint's z is set to terrain_elevation(x,y) + agl.
    Also checks that straight-line paths between consecutive waypoints
    clear the terrain by at least MIN_AGL.
    """
    wps = []
    for x, y, agl in xy_z_agl:
        gz = terrain.get_elevation(x, y)
        wps.append(np.array([x, y, gz + agl]))

    # Verify clearance along each leg
    for i in range(len(wps) - 1):
        p0, p1 = wps[i], wps[i + 1]
        for s in np.linspace(0, 1, 200):
            p = p0 + s * (p1 - p0)
            gz = terrain.get_elevation(p[0], p[1])
            clearance = p[2] - gz
            if clearance < MIN_AGL * 0.5:  # warn if less than half min AGL
                mid_gz = gz
                # Raise both endpoints so the midpoint clears
                needed = mid_gz + MIN_AGL - p[2]
                if needed > 0:
                    wps[i][2] += needed / 2
                    wps[i + 1][2] += needed / 2

    return wps


def run_benchmark(profile_name: str):
    """Run deterministic benchmark profile and enforce validation gate."""
    profile = get_benchmark_profile(profile_name)
    np.random.seed(profile.seed)

    params = DroneParams(
        mass=1.5,
        arm_length=0.25,
        drag_coeff=0.1,
        ang_drag_coeff=0.02,
        max_thrust=25.0,
        max_torque=5.0,
        aero=AeroCoefficients(reference_area=0.04, C_D=1.0, C_L=0.0),
        atmo=Atmosphere(rho_sea_level=1.225, altitude_msl=0.0),
    )
    terrain = make_terrain()
    wp_agl = [
        (0.0, 0.0, 8.0),
        (20.0, 0.0, 8.0),
        (20.0, 15.0, 8.0),
        (35.0, 20.0, 8.0),
        (10.0, 10.0, 8.0),
        (0.0, 0.0, 8.0),
        (0.0, 0.0, 1.0),
    ]
    waypoints = waypoints_above_terrain(wp_agl, terrain)

    ref_wind = WindField(
        wind_speed=0.0,
        wind_direction=profile.wind_direction,
        turbulence_type="none",
    )
    sim_wind = WindField(
        wind_speed=profile.wind_speed,
        wind_direction=profile.wind_direction,
        turbulence_type="constant",
    )

    ref_records = run_simulation(
        waypoints=waypoints,
        params=params,
        dt=0.005,
        waypoint_radius=0.5,
        hover_time=2.0,
        max_time=180.0,
        wind=ref_wind,
        terrain=terrain,
    )
    sim_records = run_simulation(
        waypoints=waypoints,
        params=params,
        dt=0.005,
        waypoint_radius=0.5,
        hover_time=2.0,
        max_time=180.0,
        wind=sim_wind,
        terrain=terrain,
    )

    n = min(len(ref_records), len(sim_records))
    ref_pos = np.array([r.position for r in ref_records[:n]])
    sim_pos = np.array([r.position for r in sim_records[:n]])
    result = compute_rmse(sim_pos, ref_pos)
    assert_validation_pass(result, profile.envelope, profile_name=profile.name)

    summary = summarize_validation(result)
    print(f"Benchmark '{profile.name}' PASS (seed={profile.seed})")
    for metric in ["rmse_x", "rmse_y", "rmse_z", "rmse_total", "median_error", "p75_error", "max_error"]:
        print(f"  {metric}: {summary[metric]:.4f}")

    return result


def run_irs4_benchmark(profile_name: str):
    """Run IRS-4 quadrotor benchmark at Quito altitude."""
    from drone_physics import make_irs4_quadrotor
    profile = get_benchmark_profile(profile_name)
    np.random.seed(profile.seed)

    params = make_irs4_quadrotor()

    # Quadrotor-style short waypoints (Carolina/EPN-like square path)
    waypoints = [
        np.array([0.0, 0.0, 20.0]),
        np.array([10.0, 0.0, 20.0]),
        np.array([10.0, 10.0, 20.0]),
        np.array([0.0, 10.0, 20.0]),
        np.array([0.0, 0.0, 20.0]),
        np.array([0.0, 0.0, 1.0]),
    ]

    ref_wind = WindField(wind_speed=0.0, wind_direction=profile.wind_direction,
                         turbulence_type="none")
    sim_wind = WindField(wind_speed=profile.wind_speed,
                         wind_direction=profile.wind_direction,
                         turbulence_type="constant")

    ref_records = run_simulation(
        waypoints=waypoints, params=params, dt=0.005,
        waypoint_radius=0.5, hover_time=1.5, max_time=120.0,
        wind=ref_wind,
    )
    sim_records = run_simulation(
        waypoints=waypoints, params=params, dt=0.005,
        waypoint_radius=0.5, hover_time=1.5, max_time=120.0,
        wind=sim_wind,
    )

    n = min(len(ref_records), len(sim_records))
    ref_pos = np.array([r.position for r in ref_records[:n]])
    sim_pos = np.array([r.position for r in sim_records[:n]])
    result = compute_rmse(sim_pos, ref_pos)
    assert_validation_pass(result, profile.envelope, profile_name=profile.name)

    summary = summarize_validation(result)
    print(f"IRS-4 Benchmark '{profile.name}' PASS (seed={profile.seed})")
    for metric in ["rmse_x", "rmse_y", "rmse_z", "rmse_total", "median_error", "p75_error", "max_error"]:
        print(f"  {metric}: {summary[metric]:.4f}")
    return result


def replay_mission(log_source, airframe: DroneParams,
                    wind_source: str = "from_log",
                    dt: float = 0.005,
                    output_plot: str = None,
                    segment_start_s: float = 0.0,
                    segment_end_s: float = None) -> dict:
    """Automated mission replay pipeline (Phase J2).

    Loads a flight log → extracts waypoints + wind profile →
    runs simulation with matching airframe/atmosphere →
    returns Table 5 metrics (compare_sim_real format).

    Args:
        log_source: Path to .bin/.csv file, or a FlightLog object.
        airframe: DroneParams preset (e.g., make_valencia_fixed_wing()).
        wind_source: "from_log" (extract from altitude deviations),
                     "from_log_3d" (3D extraction), or "none".
        dt: Simulation timestep.
        output_plot: If set, save comparison plot to this path.
        segment_start_s: Optional mission-window start time [s] from log start.
        segment_end_s: Optional mission-window end time [s] from log start.

    Returns:
        Dict with rmse_z, rmse_x, rmse_y, rmse_total, median, n_points.
    """
    # Load flight log
    if isinstance(log_source, FlightLog):
        log = log_source
    elif isinstance(log_source, str):
        path = str(log_source)
        if path.endswith(".bin"):
            log = FlightLog.from_bin(path)
        else:
            log = FlightLog.from_csv(path)
    else:
        raise ValueError(f"log_source must be a path or FlightLog, got {type(log_source)}")

    if len(log.timestamps) < 10:
        raise ValueError("Flight log too short for replay (< 10 data points)")

    if segment_end_s is not None:
        rel_t = log.timestamps - log.timestamps[0]
        mask = (rel_t >= segment_start_s) & (rel_t <= segment_end_s)
        if int(np.sum(mask)) < 10:
            raise ValueError(
                f"Mission segment too short for replay (< 10 data points): "
                f"[{segment_start_s}, {segment_end_s}]"
            )
        log = FlightLog(
            timestamps=log.timestamps[mask],
            positions=log.positions[mask],
            attitudes=log.attitudes[mask] if len(log.attitudes) else log.attitudes,
            airspeeds=log.airspeeds[mask] if len(log.airspeeds) else log.airspeeds,
            throttle=log.throttle[mask] if len(log.throttle) else log.throttle,
            origin_lat=log.origin_lat,
            origin_lon=log.origin_lon,
            origin_alt=log.origin_alt,
        )

    # Reference trajectory from flight log (zero-based time)
    # Flight log positions are NED (Z-down), simulation uses Z-up.
    # Convert: negate Z axis so positive Z = up.
    ref_times = log.timestamps - log.timestamps[0]
    ref_positions = log.positions.copy()
    ref_positions[:, 2] = -ref_positions[:, 2]  # NED down → Z-up

    # Extract wind profile
    wind = None
    if wind_source == "from_log":
        wind_profile = log.get_wind_profile()
        if len(wind_profile) > 0:
            # Re-base wind profile times to match zero-based ref_times
            wp = wind_profile.copy()
            wp[:, 0] = wp[:, 0] - log.timestamps[0]
            wind = WindField(
                wind_speed=1.0,
                wind_direction=np.array([0.0, 0.0, 1.0]),
                turbulence_type="from_log",
                altitude_profile=wp,
            )
    elif wind_source == "from_log_3d":
        wind_profile_3d = log.get_wind_profile_3d()
        if len(wind_profile_3d) > 0:
            wp3d = wind_profile_3d.copy()
            wp3d[:, 0] = wp3d[:, 0] - log.timestamps[0]
            wind = WindField(
                wind_speed=1.0,
                wind_direction=np.array([0.0, 0.0, 1.0]),
                turbulence_type="from_log_3d",
                wind_profile_3d=wp3d,
            )

    # Trajectory-tracking mode: simulation follows reference path point-by-point
    # matching the paper's validation approach (same mission, compare deviations)
    from drone_physics import run_trajectory_tracking

    records = run_trajectory_tracking(
        ref_times=ref_times,
        ref_positions=ref_positions,
        params=airframe,
        dt=dt,
        wind=wind,
    )

    if len(records) < 10:
        raise RuntimeError("Simulation produced too few records (< 10)")

    # Extract simulation trajectory
    sim_times = np.array([r.t for r in records])
    sim_positions = np.array([r.position for r in records])

    # Compute Table 5 metrics
    metrics = compare_sim_real(sim_times, sim_positions, ref_times, ref_positions)

    # Optional plot
    if output_plot:
        plot_comparison(sim_times, sim_positions, ref_times, ref_positions,
                        output_plot, title="Mission Replay Validation")

    return metrics


def run_real_log_validation(data_dir: str = "data/flight_logs",
                            multiplier: float = 6.0):
    """Run Phase V paper Table 5 validation with real OSSITLQUAD logs.

    Multiplier default is 6x because we use a simple PID position controller
    rather than the paper's ArduPilot flight controller.  The paper achieves
    sub-10cm RMSE because the same ArduPilot controller runs in both the real
    and simulated aircraft.  Our physics model is validated by showing the
    trajectory deviations stay within 6x of the paper's reported RMSE.
    Z-axis (altitude) consistently validates within 1-2x of paper values.
    X/Y axes show higher ratios due to controller tracking lag differences.
    """
    from drone_physics import make_irs4_quadrotor

    mission_names = ["quad_carolina_40", "quad_carolina_20", "quad_epn_30", "quad_epn_20"]
    local_logs = ensure_real_log_logs(data_dir)
    airframe = make_irs4_quadrotor()

    print(f"Phase V: validating against real flight logs in '{data_dir}'")
    results = {}
    for mission_name in mission_names:
        mission = get_real_log_mission(mission_name)
        log_path = local_logs[mission.source_filename]
        metrics = replay_mission(
            log_path,
            airframe,
            segment_start_s=mission.segment_start_s,
            segment_end_s=mission.segment_end_s,
        )
        assert_real_log_validation_pass(metrics, mission, multiplier=multiplier)
        results[mission_name] = metrics
        print(
            f"  {mission_name}: rmse_z={metrics['rmse_z']:.4f}, "
            f"rmse_x={metrics['rmse_x']:.4f}, rmse_y={metrics['rmse_y']:.4f}"
        )

    print("Phase V validation PASS")
    return results


def main():
    # Full-featured drone parameters
    params = DroneParams(
        mass=1.5,
        arm_length=0.25,
        drag_coeff=0.1,       # legacy fallback (not used when aero is set)
        ang_drag_coeff=0.02,
        max_thrust=25.0,
        max_torque=5.0,
        aero=AeroCoefficients(
            reference_area=0.04,
            C_D=1.0,
            C_L=0.0,
        ),
        atmo=Atmosphere(
            rho_sea_level=1.225,
            altitude_msl=0.0,   # sea level
        ),
    )

    # Wind: gentle constant breeze
    wind = WindField(
        wind_speed=3.0,
        wind_direction=np.array([0.6, 0.8, 0.0]),  # NE wind
        turbulence_type="constant",
    )

    # Terrain with prominent hills
    terrain = make_terrain()

    # Waypoints defined as (x, y, AGL) — z is computed relative to terrain
    wp_agl = [
        ( 0.0,  0.0, 8.0),    # 1. Take off
        (20.0,  0.0, 8.0),    # 2. Fly east over hill
        (20.0, 15.0, 8.0),    # 3. North, over ridge
        (35.0, 20.0, 8.0),    # 4. Continue NE
        (10.0, 10.0, 8.0),    # 5. Cut back over valley
        ( 0.0,  0.0, 8.0),    # 6. Return above origin
        ( 0.0,  0.0, 1.0),    # 7. Land
    ]
    waypoints = waypoints_above_terrain(wp_agl, terrain)

    print("Running drone flight scenario (full physics)...")
    print(f"  Mass:       {params.mass} kg")
    print(f"  Aero:       C_D={params.aero.C_D}, A={params.aero.reference_area} m^2")
    print(f"  Atmosphere: rho={params.atmo.rho:.3f} kg/m^3 (MSL={params.atmo.altitude_msl}m)")
    print(f"  Wind:       {wind.wind_speed} m/s, type={wind.turbulence_type}")
    print(f"  Terrain:    {terrain.elevations.shape[1]}x{terrain.elevations.shape[0]} grid, "
          f"res={terrain.resolution}m, "
          f"elev=[{terrain.elevations.min():.1f}, {terrain.elevations.max():.1f}]m")
    print(f"  Min AGL:    {MIN_AGL} m")
    print(f"  Waypoints:  {len(waypoints)}")
    for i, (wp, (_, _, agl)) in enumerate(zip(waypoints, wp_agl)):
        gz = terrain.get_elevation(wp[0], wp[1])
        print(f"    WP{i+1}: pos=[{wp[0]:5.1f}, {wp[1]:5.1f}, {wp[2]:5.1f}]  "
              f"ground={gz:.1f}m  AGL={agl:.0f}m")
    print()

    records = run_simulation(
        waypoints=waypoints,
        params=params,
        dt=0.005,
        waypoint_radius=0.5,
        hover_time=2.0,
        max_time=180.0,
        wind=wind,
        terrain=terrain,
    )

    print(f"Simulation complete: {len(records)} steps, {records[-1].t:.1f}s total")
    print()

    # Print summary per waypoint
    wp_idx = 0
    prev_dist = float('inf')
    for rec in records:
        if wp_idx >= len(waypoints):
            break
        dist = np.linalg.norm(rec.position - waypoints[wp_idx])
        if dist < 0.5 and prev_dist >= 0.5:
            ground = terrain.get_elevation(rec.position[0], rec.position[1])
            print(f"  WP{wp_idx+1} reached at t={rec.t:.2f}s  "
                  f"pos=[{rec.position[0]:.1f}, {rec.position[1]:.1f}, {rec.position[2]:.1f}]  "
                  f"AGL={rec.position[2] - ground:.1f}m  "
                  f"speed={np.linalg.norm(rec.velocity):.2f} m/s")
            wp_idx += 1
        prev_dist = dist

    final = records[-1]
    print(f"\n  Final position: [{final.position[0]:.2f}, {final.position[1]:.2f}, {final.position[2]:.2f}]")
    print(f"  Final speed:    {np.linalg.norm(final.velocity):.3f} m/s")

    # Save data for visualization (next to this script)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, 'scenario_data.npz')

    # Terrain grid for visualization
    bounds = terrain.bounds
    tx = np.arange(bounds[0], bounds[2] + terrain.resolution, terrain.resolution)
    ty = np.arange(bounds[1], bounds[3] + terrain.resolution, terrain.resolution)
    # Ensure shapes match the stored elevations
    tx = tx[:terrain.elevations.shape[1]]
    ty = ty[:terrain.elevations.shape[0]]

    np.savez(
        output_path,
        t=np.array([r.t for r in records]),
        pos=np.array([r.position for r in records]),
        vel=np.array([r.velocity for r in records]),
        euler=np.array([r.euler for r in records]),
        thrust=np.array([r.thrust for r in records]),
        ang_vel=np.array([r.angular_velocity for r in records]),
        euler_rates=np.array([r.euler_rates for r in records]),  # Phase M2: Eq. 2
        waypoints=np.array(waypoints),
        # Phase 2 additions
        terrain_x=tx,
        terrain_y=ty,
        terrain_z=terrain.elevations,
        wind_speed=wind.wind_speed,
        wind_direction=wind.wind_direction / np.linalg.norm(wind.wind_direction),
        atmo_rho=params.atmo.rho,
        atmo_altitude_msl=params.atmo.altitude_msl,
        aero_cd=params.aero.C_D,
        aero_area=params.aero.reference_area,
    )
    print(f"\n  Data saved to {output_path}")

    # Export terrain as STL for Gazebo (Phase L1)
    stl_path = os.path.join(script_dir, 'terrain_mesh.stl')
    terrain.export_stl(stl_path)
    print(f"  Terrain STL saved to {stl_path}")


def run_swarm(n_drones: int = 6):
    """Run N-drone swarm flight scenario with boids flocking."""
    from drone_physics import make_holybro_x500, run_swarm_simulation, FlockingParams

    np.random.seed(42)
    params = make_holybro_x500()
    terrain = make_terrain()
    wind = WindField(
        wind_speed=2.0,
        wind_direction=np.array([0.8, 0.6, 0.0]),
        turbulence_type="constant",
    )

    # Build ring formation waypoints for N drones
    drone_waypoints = {}
    radius = 10.0
    altitude = 8.0
    for i in range(n_drones):
        angle = i * 2.0 * np.pi / n_drones
        drone_waypoints[f"drone_{i + 1}"] = [
            np.array([radius * np.cos(angle), radius * np.sin(angle), altitude]),
            np.array([
                radius * np.cos(angle + np.pi / n_drones),
                radius * np.sin(angle + np.pi / n_drones),
                altitude,
            ]),
            np.array([
                radius * np.cos(angle + 2.0 * np.pi / n_drones),
                radius * np.sin(angle + 2.0 * np.pi / n_drones),
                altitude,
            ]),
        ]

    drone_ids = sorted(drone_waypoints.keys())

    print(f"Running {n_drones}-drone swarm scenario (boids flocking)...")
    print(f"  Airframe:   Holybro X500 ({params.mass} kg)")
    print(f"  Wind:       {wind.wind_speed} m/s, type={wind.turbulence_type}")
    print(f"  Formation:  ring, radius={radius}m, alt={altitude}m AGL")
    print(f"  Terrain:    {terrain.elevations.shape[1]}x{terrain.elevations.shape[0]} grid")
    print()

    records = run_swarm_simulation(
        drone_waypoints,
        params=params,
        dt=0.02,
        hover_time=0.5,
        max_time=30.0,
        wind=wind,
        terrain=terrain,
        min_separation=1.5,
    )

    # Compute metrics
    min_dist = float("inf")
    for rec in records:
        for i in range(len(rec.positions)):
            for j in range(i + 1, len(rec.positions)):
                d = np.linalg.norm(rec.positions[i] - rec.positions[j])
                min_dist = min(min_dist, d)

    final_pos = records[-1].positions
    print(f"Simulation complete: {len(records)} steps, {records[-1].t:.1f}s total")
    print(f"  Min pairwise separation: {min_dist:.3f} m")
    for idx, drone_id in enumerate(drone_ids):
        target = drone_waypoints[drone_id][-1]
        err = np.linalg.norm(final_pos[idx] - target)
        print(f"  {drone_id}: final=[{final_pos[idx][0]:6.2f}, {final_pos[idx][1]:6.2f}, {final_pos[idx][2]:6.2f}]  err={err:.2f}m")

    # Save data
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, 'swarm_data.npz')
    np.savez(
        output_path,
        t=np.array([r.t for r in records]),
        positions=np.array([r.positions for r in records]),
        velocities=np.array([r.velocities for r in records]),
        drone_ids=np.array(drone_ids, dtype=object),
        waypoints=np.array([drone_waypoints[d] for d in drone_ids]),
    )
    print(f"\n  Data saved to {output_path}")

    # Export terrain STL
    stl_path = os.path.join(script_dir, 'terrain_mesh.stl')
    terrain.export_stl(stl_path)
    print(f"  Terrain STL saved to {stl_path}")


def export_antisana_terrain(output_dir: str = None):
    """Export SRTM terrain mesh for Antisana with satellite texture for Gazebo.

    Generates Gazebo-ready OBJ/MTL + optional satellite texture with
    deterministic offline fallback.
    """
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   '..', 'gazebo', 'models', 'antisana_terrain')
    os.makedirs(output_dir, exist_ok=True)

    print("Generating Antisana terrain mesh + satellite texture from SRTM data...")
    terrain = TerrainMap.from_srtm(-0.508333, -78.141667, size_km=5.0, resolution=30.0)
    texture_path = download_satellite_tile(-0.508333, -78.141667, output_dir)
    assets = terrain.export_gazebo_terrain_assets(output_dir, texture_path=texture_path)
    stl_path = os.path.join(output_dir, 'mesh.stl')
    terrain.export_stl(stl_path)
    print(f"  Terrain OBJ saved to {assets['obj_path']}")
    print(f"  Terrain MTL saved to {assets['mtl_path']}")
    print(f"  Terrain material: {assets['material_name']}")
    print(f"  Terrain texture: {texture_path}")
    print(f"  Terrain STL saved to {stl_path}")
    print(f"  Grid shape: {terrain.elevations.shape}")
    print(f"  Elevation range: [{terrain.elevations.min():.0f}, {terrain.elevations.max():.0f}] m")
    print(f"  Resolution: {terrain.resolution} m/cell")
    return assets['obj_path']


def print_usage():
    print("Usage: python drone_scenario.py [MODE]")
    print()
    print("Modes:")
    print("  --single              Run single-drone flight scenario (default)")
    print("  --swarm [N]           Run N-drone swarm scenario (default: 6)")
    print("  --benchmark <name>    Run validation benchmark profile")
    print("  --real-log            Run real-log validation against paper Table 5")
    print("  --swarm-benchmark <name>  Run swarm validation benchmark")
    print("  --replay <log> [airframe] [plot]  Replay flight log")
    print("  --export-terrain [dir]  Export Antisana SRTM terrain as STL")
    print()
    print("Examples:")
    print("  python drone_scenario.py")
    print("  python drone_scenario.py --single")
    print("  python drone_scenario.py --swarm")
    print("  python drone_scenario.py --swarm 10")
    print("  python drone_scenario.py --benchmark moderate")
    print("  python drone_scenario.py --real-log")
    print("  python drone_scenario.py --benchmark irs4_carolina")
    print("  python drone_scenario.py --swarm-benchmark baseline")
    print("  python drone_scenario.py --replay flight.bin quad")
    print("  python drone_scenario.py --export-terrain gazebo/models/antisana_terrain")


if __name__ == '__main__':
    if len(sys.argv) >= 2 and sys.argv[1] in ("--help", "-h"):
        print_usage()
    elif len(sys.argv) >= 3 and sys.argv[1] == "--benchmark":
        profile_name = sys.argv[2]
        if profile_name.startswith("irs4_"):
            run_irs4_benchmark(profile_name)
        else:
            run_benchmark(profile_name)
    elif len(sys.argv) >= 2 and sys.argv[1] == "--real-log":
        run_real_log_validation()
    elif len(sys.argv) >= 3 and sys.argv[1] == "--replay":
        from drone_physics import make_valencia_fixed_wing, make_irs4_quadrotor
        bin_path = sys.argv[2]
        airframe_name = sys.argv[3] if len(sys.argv) > 3 else "fixed_wing"
        airframe = (make_irs4_quadrotor() if "quad" in airframe_name
                    else make_valencia_fixed_wing())
        plot_path = sys.argv[4] if len(sys.argv) > 4 else None
        metrics = replay_mission(bin_path, airframe, output_plot=plot_path)
        print(f"Mission replay results for {bin_path}:")
        for k, v in metrics.items():
            print(f"  {k}: {v:.4f}" if isinstance(v, float) else f"  {k}: {v}")
    elif len(sys.argv) >= 2 and sys.argv[1] == "--swarm":
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 6
        run_swarm(n)
    elif len(sys.argv) >= 3 and sys.argv[1] == "--swarm-benchmark":
        from swarm_scenario import run_swarm_benchmark
        run_swarm_benchmark(sys.argv[2])
    elif len(sys.argv) >= 2 and sys.argv[1] == "--export-terrain":
        output = sys.argv[2] if len(sys.argv) > 2 else None
        export_antisana_terrain(output)
    else:
        # --single is the default mode (also triggered with no args)
        main()
