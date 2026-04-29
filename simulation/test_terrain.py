"""
Tests: Terrain

Split from `test_drone_physics.py` so the per-domain test surface is
navigable. Helpers live in `_test_common.py`.
"""

import numpy as np
import pytest
import warnings
from urllib.error import HTTPError
from pathlib import Path

from drone_physics import (
    DroneParams, DroneState, DroneCommand,
    PositionController, PIDController,
    physics_step, euler_to_rotation, rotation_to_euler,
    euler_rates_from_body_rates,
    run_simulation, run_trajectory_tracking, GRAVITY,
    AeroCoefficients, Atmosphere, _compute_quadratic_drag,
    _compute_lift, compute_aoa,
    FixedWingAero, QuadrotorAero, MotorModel, BatteryModel,
    make_generic_quad, make_holybro_x500, make_fixed_wing,
    make_valencia_fixed_wing, make_irs4_quadrotor,
    run_swarm_simulation, calculate_flocking_vector, FlockingParams,
)
from wind_model import WindField
from terrain import TerrainMap

from _test_common import (
    PARITY_MAX_DELTA_M,
    PARITY_RNG_SEED,
    PARITY_SAMPLE_COUNT,
    PROFILE_BASE_SPEED_MS,
    CRUISE_AGL_FLOOR_M,
    CRUISE_ALTITUDE_M,
    CLIMB_TIMEOUT_S,
    live_js_source,
    parity_entry_names,
    ramp_terrain,
    regression_mission,
    stress_mission,
    wind_profile_names_safe,
)


# Aliases: keep the underscore-prefixed call sites that the test bodies
# carry over from the original `test_drone_physics.py`.
_LIVE_JS = (Path(__file__).resolve().parent
            / "runtime_view" / "web" / "live.js")
_live_js_source = live_js_source
_parity_entry_names = parity_entry_names
_ramp_terrain = ramp_terrain
_regression_mission = regression_mission
_wind_profile_names = wind_profile_names_safe
_stress_mission = stress_mission
_PROFILE_BASE_SPEED_MS = PROFILE_BASE_SPEED_MS


class TestTerrain:
    def test_flat_terrain_elevation(self):
        """Flat terrain should return constant elevation everywhere."""
        t = TerrainMap.flat(elevation=50.0, size=200.0, resolution=10.0)
        np.testing.assert_allclose(t.get_elevation(0, 0), 50.0)
        np.testing.assert_allclose(t.get_elevation(99, -50), 50.0)
        np.testing.assert_allclose(t.get_elevation(-100, 100), 50.0)

    def test_from_array_elevation_query(self):
        """Elevation query should interpolate grid values correctly."""
        # 3x3 grid: center cell is a hill
        grid = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0],
        ])
        t = TerrainMap.from_array(grid, origin=(0.0, 0.0), resolution=1.0)
        # Exact grid point (1,1) → 10.0
        np.testing.assert_allclose(t.get_elevation(1.0, 1.0), 10.0)
        # Corner (0,0) → 0.0
        np.testing.assert_allclose(t.get_elevation(0.0, 0.0), 0.0)
        # Midpoint (0.5, 1.0) → bilinear interp of 0 and 10 → 5.0
        np.testing.assert_allclose(t.get_elevation(0.5, 1.0), 5.0, atol=0.01)

    def test_terrain_collision_detection(self):
        """check_collision should detect when position is below terrain."""
        grid = np.array([
            [5.0, 5.0],
            [5.0, 5.0],
        ])
        t = TerrainMap.from_array(grid, origin=(0.0, 0.0), resolution=10.0)
        assert t.check_collision(np.array([5.0, 5.0, 4.0])) == True
        assert t.check_collision(np.array([5.0, 5.0, 5.0])) == True
        assert t.check_collision(np.array([5.0, 5.0, 6.0])) == False

    def test_terrain_in_physics_step(self):
        """Drone should not fall below terrain surface."""
        # Hill at 20m elevation
        grid = np.full((10, 10), 20.0)
        t = TerrainMap.from_array(grid, origin=(-50.0, -50.0), resolution=10.0)

        params = DroneParams()
        state = DroneState(position=np.array([0.0, 0.0, 25.0]))
        cmd = DroneCommand(thrust=0.0)  # no thrust, will fall

        for _ in range(2000):
            state = physics_step(state, cmd, params, dt=0.005, terrain=t)

        # Should have landed on terrain at z=20, not z=0
        assert state.position[2] >= 20.0
        np.testing.assert_allclose(state.position[2], 20.0, atol=0.1)

    def test_from_function_terrain(self):
        """Terrain from analytical function should give correct elevations."""
        # Simple slope: z = 0.1 * x
        t = TerrainMap.from_function(
            lambda x, y: 0.1 * x,
            x_range=(0, 100), y_range=(0, 100), resolution=1.0,
        )
        np.testing.assert_allclose(t.get_elevation(50.0, 50.0), 5.0, atol=0.1)
        np.testing.assert_allclose(t.get_elevation(0.0, 0.0), 0.0, atol=0.1)
        np.testing.assert_allclose(t.get_elevation(100.0, 0.0), 10.0, atol=0.1)

    def test_stl_file_not_found(self):
        """Loading a nonexistent STL should raise FileNotFoundError."""
        with pytest.raises(FileNotFoundError):
            TerrainMap.from_stl("/nonexistent/terrain.stl")


# ── Fixed-Wing Aerodynamics (AoA / Stall) ────────────────────────────────────


class TestTerrainSRTM:
    def test_from_hgt_synthetic(self, tmp_path):
        """from_hgt should load a synthetic .hgt file and return terrain."""
        # Create synthetic SRTM3 tile (1201x1201, big-endian int16)
        size = 1201
        grid = np.full((size, size), 4500, dtype='>i2')
        hgt_path = str(tmp_path / "S01W079.hgt")
        with open(hgt_path, 'wb') as f:
            f.write(grid.tobytes())

        terrain = TerrainMap.from_hgt(hgt_path, lat=-0.5, lon=-78.5, size_km=2.0)
        assert terrain.origin_gps is not None
        assert terrain.elevations.shape[0] > 1
        assert terrain.elevations.shape[1] > 1
        # Elevation should be ~4500m
        center_elev = terrain.get_elevation(0, 0)
        assert 4000 < center_elev < 5000

    def test_gps_elevation_query(self):
        """get_elevation_gps should convert lat/lon to local coords."""
        # Create terrain with known GPS origin
        grid = np.full((10, 10), 100.0)
        terrain = TerrainMap(
            elevations=grid,
            origin=(-500, -500),
            resolution=100.0,
            origin_gps=(-0.5, -78.0, 100.0),
        )
        # Query at origin should return ~100m
        elev = terrain.get_elevation_gps(-0.5, -78.0)
        assert 90 < elev < 110

    def test_gps_query_without_origin_raises(self):
        """get_elevation_gps without origin_gps should raise ValueError."""
        terrain = TerrainMap.flat(100.0)
        with pytest.raises(ValueError, match="origin_gps not set"):
            terrain.get_elevation_gps(-0.5, -78.0)

    def test_hgt_parser_file_not_found(self):
        """Missing .hgt file should raise FileNotFoundError."""
        with pytest.raises(FileNotFoundError):
            TerrainMap.from_hgt("/nonexistent/S01W079.hgt", lat=-0.5, lon=-78.5)


# ── Gazebo Model Validation ──────────────────────────────────────────────────


class TestTerrainSTLExport:
    """Tests for TerrainMap.export_stl() (binary STL generation)."""

    def test_export_stl_creates_file(self, tmp_path):
        """export_stl writes a valid binary STL file."""
        terrain = TerrainMap.from_array(
            np.array([[0.0, 1.0, 2.0],
                      [1.0, 2.0, 3.0],
                      [2.0, 3.0, 4.0]]),
            resolution=10.0,
        )
        stl_path = str(tmp_path / "test.stl")
        terrain.export_stl(stl_path)

        import struct
        data = open(stl_path, 'rb').read()
        assert len(data) >= 84, "STL file too small"

        n_triangles = struct.unpack_from('<I', data, 80)[0]
        # 3x3 grid → 2x2 cells → 4 cells × 2 triangles = 8
        assert n_triangles == 8

    def test_export_stl_expected_size(self, tmp_path):
        """STL file size matches: 84 + n_triangles * 50."""
        import struct
        elev = np.random.rand(5, 7) * 100
        terrain = TerrainMap.from_array(elev, resolution=5.0)
        stl_path = str(tmp_path / "rand.stl")
        terrain.export_stl(stl_path)

        data = open(stl_path, 'rb').read()
        n_tri = struct.unpack_from('<I', data, 80)[0]
        expected = 84 + n_tri * 50
        assert len(data) == expected
        assert n_tri == (7 - 1) * (5 - 1) * 2

    def test_export_stl_roundtrip(self, tmp_path):
        """export → from_stl preserves elevation within 1m."""
        elev = np.array([[0.0, 10.0, 20.0],
                         [5.0, 15.0, 25.0],
                         [10.0, 20.0, 30.0]])
        terrain = TerrainMap.from_array(elev, resolution=10.0)
        stl_path = str(tmp_path / "roundtrip.stl")
        terrain.export_stl(stl_path)

        loaded = TerrainMap.from_stl(stl_path, resolution=10.0)
        # Center elevation should be close to 15.0
        center_z = loaded.get_elevation(10.0, 10.0)
        assert abs(center_z - 15.0) < 2.0, f"Roundtrip center: {center_z}"

    def test_export_stl_with_scale(self, tmp_path):
        """Scale factor multiplies coordinates."""
        import struct
        elev = np.array([[0.0, 1.0], [1.0, 2.0]])
        terrain = TerrainMap.from_array(elev, resolution=1.0)
        stl_path = str(tmp_path / "scaled.stl")
        terrain.export_stl(stl_path, scale=10.0)

        data = open(stl_path, 'rb').read()
        # Read first triangle's first vertex (after normal)
        vx, vy, vz = struct.unpack_from('<fff', data, 84 + 12)
        # Origin is (0,0) with scale=10 → vertex at (0, 0, 0*10=0)
        assert abs(vz) < 0.01

    def test_export_stl_rejects_1x1(self):
        """1x1 grid cannot be tessellated."""
        terrain = TerrainMap.from_array(np.array([[5.0]]))
        with pytest.raises(ValueError, match="2x2"):
            terrain.export_stl("/dev/null")

    def test_export_stl_normals_point_up(self, tmp_path):
        """Triangle normals on flat terrain should point roughly upward."""
        import struct
        elev = np.zeros((3, 3))
        terrain = TerrainMap.from_array(elev, resolution=1.0)
        stl_path = str(tmp_path / "flat.stl")
        terrain.export_stl(stl_path)

        data = open(stl_path, 'rb').read()
        offset = 84
        for _ in range(8):
            nx, ny, nz = struct.unpack_from('<fff', data, offset)
            assert nz > 0.99, f"Normal not upward: ({nx},{ny},{nz})"
            offset += 50


# ── Terrain Coloring (Gazebo Materials) ──────────────────────────────────────


class TestTerrainColoring:
    """Tests for Gazebo terrain material files (color, texture)."""

    def test_material_file_exists(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_terrain.material')
        assert os.path.exists(path), "Material script missing"

    def test_vertex_shader_exists(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_height_color.vert')
        assert os.path.exists(path), "Vertex shader missing"

    def test_fragment_shader_exists(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_height_color.frag')
        assert os.path.exists(path), "Fragment shader missing"

    def test_material_references_shaders(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_terrain.material')
        content = open(path).read()
        assert 'antisana_height_color.vert' in content
        assert 'antisana_height_color.frag' in content
        assert 'AntisanaTerrain/HeightColored' in content

    def test_fragment_has_elevation_bands(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'media', 'materials', 'scripts',
                            'antisana_height_color.frag')
        content = open(path).read()
        assert 'green_max' in content
        assert 'brown_max' in content
        assert 'snow_min' in content

    def test_world_references_material(self):
        import os
        path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                            'worlds', 'antisana.world')
        content = open(path).read()
        assert 'AntisanaTerrain/HeightColored' in content
        assert 'antisana_terrain.material' in content


class TestTerrainSatelliteTexture:
    """Tests for satellite-texture terrain pipeline (tile download, overlay)."""

    def test_satellite_tile_download_has_offline_fallback(self, tmp_path):
        from terrain import download_satellite_tile

        tile = download_satellite_tile(-0.508333, -78.141667, str(tmp_path), tile_size=64)
        assert tile is not None
        assert tile.endswith('.ppm')
        assert (tmp_path / Path(tile).name).exists()

    def test_export_obj_with_uv_contains_uv_and_faces(self, tmp_path):
        terrain = TerrainMap.from_array(
            np.array([[4400.0, 4450.0, 4500.0],
                      [4420.0, 4470.0, 4520.0],
                      [4440.0, 4490.0, 4540.0]]),
            resolution=10.0,
        )
        tex = tmp_path / 'sat.ppm'
        tex.write_bytes(b'P6\n1 1\n255\n\x00\x00\x00')
        out_obj = tmp_path / 'terrain.obj'

        terrain.export_obj_with_uv(str(out_obj), texture_path=str(tex))
        content = out_obj.read_text()
        assert 'vt ' in content
        assert 'f ' in content

        mtl = out_obj.with_suffix('.mtl').read_text()
        assert 'map_Kd sat.ppm' in mtl

    def test_export_assets_fallbacks_to_height_material_without_texture(self, tmp_path):
        terrain = TerrainMap.flat(elevation=4500.0, size=20.0, resolution=10.0)
        assets = terrain.export_gazebo_terrain_assets(str(tmp_path), texture_path=str(tmp_path / 'missing.ppm'))
        assert assets['material_name'] == 'AntisanaTerrain/HeightColored'
        assert Path(assets['obj_path']).exists()

    def test_world_and_material_include_satellite_reference(self):
        import os
        import xml.etree.ElementTree as ET
        material_path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                                     'media', 'materials', 'scripts',
                                     'antisana_terrain.material')
        world_path = os.path.join(os.path.dirname(__file__), '..', 'gazebo',
                                  'worlds', 'antisana.world')
        material = open(material_path).read()
        world = open(world_path).read()
        assert 'AntisanaTerrain/SatelliteTextured' in material
        assert 'antisana_satellite.ppm' in material
        assert 'AntisanaTerrain/SatelliteTextured' in world

        world_xml = ET.parse(world_path).getroot()
        world_node = world_xml.find('world')
        model_names = [m.get('name') for m in world_node.findall('model')]
        assert 'antisana_terrain' in model_names
        assert 'ground_plane' not in model_names


class TestTerrainParity:
    """Manifest loader + export-roundtrip parity (terrain integration)."""

    def test_load_from_manifest_unknown_name(self):
        from terrain import load_from_manifest
        with pytest.raises(ValueError, match="unknown terrain"):
            load_from_manifest("does_not_exist")

    def test_load_from_manifest_unknown_fn(self, tmp_path):
        """Unknown 'fn' under source=function must raise ValueError."""
        from terrain import load_from_manifest
        manifest = tmp_path / "manifest.toml"
        manifest.write_text(
            '[bad]\n'
            'source = "function"\n'
            'fn = "not_registered"\n'
            'x_range = [-1.0, 1.0]\n'
            'y_range = [-1.0, 1.0]\n'
            'resolution_m = 0.5\n',
            encoding="utf-8",
        )
        with pytest.raises(ValueError, match="unknown fn"):
            load_from_manifest("bad", manifest_path=manifest)

    def test_load_from_manifest_missing_file(self, tmp_path):
        """Pointing the loader at a non-existent manifest must raise."""
        from terrain import load_from_manifest
        with pytest.raises(FileNotFoundError, match="terrain manifest not found"):
            load_from_manifest("flat", manifest_path=tmp_path / "nope.toml")

    def test_load_from_manifest_missing_source(self, tmp_path):
        """Entry with no 'source' field must raise a descriptive ValueError."""
        from terrain import load_from_manifest
        manifest = tmp_path / "manifest.toml"
        manifest.write_text('[oops]\nelevation = 0.0\n', encoding="utf-8")
        with pytest.raises(ValueError,
                           match=r"missing required field 'source'"):
            load_from_manifest("oops", manifest_path=manifest)

    def test_load_from_manifest_unknown_source(self, tmp_path):
        from terrain import load_from_manifest
        manifest = tmp_path / "manifest.toml"
        manifest.write_text('[oops]\nsource = "alien"\n', encoding="utf-8")
        with pytest.raises(ValueError, match=r"unknown source 'alien'"):
            load_from_manifest("oops", manifest_path=manifest)

    def test_load_from_manifest_missing_required_field(self, tmp_path):
        """source=srtm without lat/lon must raise a descriptive ValueError."""
        from terrain import load_from_manifest
        manifest = tmp_path / "manifest.toml"
        manifest.write_text(
            '[bad]\nsource = "srtm"\nlon = 0.0\n', encoding="utf-8",
        )
        with pytest.raises(ValueError,
                           match=r"missing required field 'lat'"):
            load_from_manifest("bad", manifest_path=manifest)

    def test_load_from_manifest_array_checksum_mismatch(self, tmp_path):
        from terrain import load_from_manifest
        arr = np.array([[0.0, 1.0], [2.0, 3.0]], dtype=float)
        np.savez(tmp_path / "terrain.npz", elevations=arr)
        manifest = tmp_path / "manifest.toml"
        manifest.write_text(
            '[arr]\n'
            'source = "array"\n'
            'npz_path = "terrain.npz"\n'
            'checksum_sha256 = "deadbeef"\n',
            encoding="utf-8",
        )
        with pytest.raises(ValueError, match="checksum mismatch"):
            load_from_manifest("arr", manifest_path=manifest)

    def test_load_from_manifest_array_checksum_match(self, tmp_path):
        import hashlib
        from terrain import load_from_manifest
        arr = np.array([[0.0, 1.0], [2.0, 3.0]], dtype=float)
        p = tmp_path / "terrain.npz"
        np.savez(p, elevations=arr)
        digest = hashlib.sha256(p.read_bytes()).hexdigest()
        manifest = tmp_path / "manifest.toml"
        manifest.write_text(
            '[arr]\n'
            'source = "array"\n'
            'npz_path = "terrain.npz"\n'
            f'checksum_sha256 = "{digest}"\n',
            encoding="utf-8",
        )
        t = load_from_manifest("arr", manifest_path=manifest)
        assert t.get_elevation(0.0, 0.0) == pytest.approx(0.0)

    def test_flat_is_trivially_exact(self):
        """Flat terrain: any (x,y) returns the manifest's elevation exactly."""
        from terrain import load_from_manifest
        t = load_from_manifest("flat")
        assert t.is_flat()
        assert t.get_elevation(7.0, -3.0) == 0.0

    @pytest.mark.parametrize("name", _parity_entry_names())
    def test_export_roundtrip_parity(self, name, tmp_path):
        """get_elevation must survive the STL export → reload pipeline.

        Compares the source TerrainMap against one rebuilt from its own
        exported STL. Samples 100 deterministic points 5%-inside the
        bounds (STL edges are interpolated, not extrapolated). Asserts
        max |Δz| < PARITY_MAX_DELTA_M.
        """
        from terrain import load_from_manifest
        try:
            src = load_from_manifest(name)
        except (HTTPError, OSError) as exc:  # pragma: no cover
            pytest.skip(f"{name}: source unavailable ({type(exc).__name__})")

        if src.is_flat():
            pytest.skip(f"{name}: flat terrain — STL round-trip is trivially exact")

        stl_path = tmp_path / f"{name}.stl"
        src.export_stl(str(stl_path))
        # Reload at the source's native resolution — the round-trip we
        # want to verify is "exporting and re-importing the same grid",
        # not "supersampling 30 m SRTM data into 1 m cells".
        rt = TerrainMap.from_stl(str(stl_path), resolution=src.resolution)

        x_min, y_min, x_max, y_max = src.bounds
        margin_x = 0.05 * (x_max - x_min)
        margin_y = 0.05 * (y_max - y_min)
        rng = np.random.default_rng(PARITY_RNG_SEED)
        xs = rng.uniform(x_min + margin_x, x_max - margin_x, PARITY_SAMPLE_COUNT)
        ys = rng.uniform(y_min + margin_y, y_max - margin_y, PARITY_SAMPLE_COUNT)

        deltas = np.array([
            abs(src.get_elevation(float(x), float(y))
                - rt.get_elevation(float(x), float(y)))
            for x, y in zip(xs, ys)
        ])
        max_delta = float(deltas.max())
        rmse = float(np.sqrt(np.mean(deltas ** 2)))
        assert max_delta < PARITY_MAX_DELTA_M, (
            f"{name}: max |Δz|={max_delta:.3f} m exceeds "
            f"{PARITY_MAX_DELTA_M} m (rmse={rmse:.3f} m)"
        )


# ──────────────────────────────────────────────────────────────────────────────
# In-loop AGL enforcement + flat/rolling/steep regression suite.
#
# Three terrain profiles, one mission each. Pass criteria:
#   - no terrain collisions (AGL ever < 0)
#   - after the climb-up phase, AGL stays above a per-profile floor
#   - the in-loop TerrainMonitor and the post-hoc safety.monitor_records()
#     helper agree on the per-step elevation
# ──────────────────────────────────────────────────────────────────────────────

CRUISE_ALTITUDE_M = 25.0
CRUISE_AGL_FLOOR_M = 5.0
CLIMB_TIMEOUT_S = 6.0


def _ramp_terrain():
    """Steep east-facing ramp (10° slope) bounded ±50 m around origin."""
    from terrain import TerrainMap
    return TerrainMap.from_function(
        lambda x, y: 0.176 * x,           # tan(10°) ≈ 0.176
        x_range=(-50.0, 50.0),
        y_range=(-50.0, 50.0),
        resolution=2.0,
    )


def _regression_mission(terrain) -> list:
    """Three-waypoint patrol at CRUISE_ALTITUDE_M above the highest peak."""
    z_peak = float(np.max(terrain.elevations))
    z = z_peak + CRUISE_ALTITUDE_M
    return [
        np.array([0.0, 0.0, z]),
        np.array([20.0, 0.0, z]),
        np.array([20.0, 20.0, z]),
        np.array([0.0, 0.0, z]),
    ]


class TestTerrainRegression:
    """Flat / rolling / steep terrain mission regression."""

    @pytest.fixture(params=["flat", "rolling", "steep"])
    def terrain_profile(self, request):
        from terrain import TerrainMap, load_from_manifest
        if request.param == "flat":
            return request.param, TerrainMap.flat(elevation=0.0, size=200.0,
                                                  resolution=10.0)
        if request.param == "rolling":
            return request.param, load_from_manifest("synthetic_rolling")
        return request.param, _ramp_terrain()

    def test_mission_no_terrain_collision(self, terrain_profile):
        """Drone must never penetrate the terrain (AGL < 0) over the run."""
        from drone_physics import run_simulation, DroneParams
        from safety import TerrainMonitor

        name, terrain = terrain_profile
        mon = TerrainMonitor(terrain, min_agl=0.0)
        records = run_simulation(
            waypoints=_regression_mission(terrain),
            params=DroneParams(),
            dt=0.02,
            waypoint_radius=1.0,
            hover_time=0.3,
            max_time=60.0,
            terrain=terrain,
            terrain_monitor=mon,
        )
        assert records, f"{name}: no records produced"
        assert mon.terrain_collision_count == 0, (
            f"{name}: {mon.terrain_collision_count} terrain collisions, "
            f"min AGL observed {mon.min_agl_observed:.2f} m"
        )

    def test_mission_cruise_agl_above_floor(self, terrain_profile):
        """After the climb-up phase, AGL must stay above the cruise floor."""
        from drone_physics import run_simulation, DroneParams
        from safety import monitor_records

        name, terrain = terrain_profile
        records = run_simulation(
            waypoints=_regression_mission(terrain),
            params=DroneParams(),
            dt=0.02,
            waypoint_radius=1.0,
            hover_time=0.3,
            max_time=60.0,
            terrain=terrain,
        )
        cruise_records = [r for r in records if r.t >= CLIMB_TIMEOUT_S]
        assert cruise_records, (
            f"{name}: simulation ended before climb-up timeout "
            f"({CLIMB_TIMEOUT_S}s)"
        )
        mon = monitor_records(cruise_records, terrain, min_agl=CRUISE_AGL_FLOOR_M)
        assert mon.clearance_violation_count == 0, (
            f"{name}: {mon.clearance_violation_count} clearance violations "
            f"during cruise, min AGL {mon.min_agl_observed:.2f} m "
            f"(floor {CRUISE_AGL_FLOOR_M:.1f} m)"
        )

    def test_inloop_monitor_matches_post_hoc(self, terrain_profile):
        """In-loop TerrainMonitor.check must agree with the post-hoc walker."""
        from drone_physics import run_simulation, DroneParams
        from safety import TerrainMonitor, monitor_records

        name, terrain = terrain_profile
        inloop = TerrainMonitor(terrain, min_agl=2.0)
        records = run_simulation(
            waypoints=_regression_mission(terrain),
            params=DroneParams(),
            dt=0.02,
            waypoint_radius=1.0,
            hover_time=0.3,
            max_time=60.0,
            terrain=terrain,
            terrain_monitor=inloop,
        )
        post = monitor_records(records, terrain, min_agl=2.0)
        assert inloop._samples == post._samples, (
            f"{name}: sample-count mismatch in-loop={inloop._samples} "
            f"post-hoc={post._samples}"
        )
        assert inloop.min_agl_observed == pytest.approx(post.min_agl_observed), (
            f"{name}: min-AGL mismatch in-loop={inloop.min_agl_observed:.4f} "
            f"post-hoc={post.min_agl_observed:.4f}"
        )
        assert inloop.terrain_collision_count == post.terrain_collision_count
        assert inloop.clearance_violation_count == post.clearance_violation_count


# ──────────────────────────────────────────────────────────────────────────────
# Wind manifest + deterministic Dryden + spatial gradient + stress
# envelopes + per-step wind logging in SimRecord.
# ──────────────────────────────────────────────────────────────────────────────


def _wind_profile_names() -> list:
    from wind_model import wind_profile_names
    try:
        return wind_profile_names()
    except FileNotFoundError:  # pragma: no cover
        return []


# Per-profile expected wind-velocity magnitude (steady-state base wind, no
# turbulence component). Sourced from todo/wind_simulation.md.
_PROFILE_BASE_SPEED_MS = {
    "calm": 0.5,
    "crosswind": 5.0,
    "gusty": 6.0,
    "storm": 12.0,
}


def _stress_mission(z: float = 10.0):
    return [
        np.array([0.0, 0.0, z]),
        np.array([15.0, 0.0, z]),
        np.array([0.0, 0.0, z]),
    ]


class TestGazeboTerrainEmulator:
    """File-level parity vs Gazebo's heightmap algorithm."""

    @pytest.mark.parametrize("name", _parity_entry_names())
    def test_terrain_matches_gazebo_emulator(self, name):
        from gz_terrain_emulator import parity_samples
        from terrain import load_from_manifest
        try:
            terrain = load_from_manifest(name)
        except (HTTPError, OSError) as exc:  # pragma: no cover
            pytest.skip(f"{name}: source unavailable ({type(exc).__name__})")
        if terrain.is_flat():
            pytest.skip(f"{name}: flat terrain — emulator parity is trivially exact")
        _, max_delta, rmse = parity_samples(terrain, n=100)
        assert max_delta < 0.5, (
            f"{name}: max |Δz| {max_delta:.3f} m > 0.5 m vs Gazebo emulator "
            f"(rmse={rmse:.3f} m)"
        )
