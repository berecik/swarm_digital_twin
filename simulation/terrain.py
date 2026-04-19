"""
Terrain Height Map - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app

Terrain elevation model for the physics engine.
Supports: flat ground, grid arrays, STL meshes, SRTM download.
Based on Valencia et al. (2025) pipeline: SRTM -> BlenderGIS -> STL -> Gazebo.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple
import struct
import hashlib
from pathlib import Path
import shutil


R_EARTH = 6371000.0  # Earth radius [m]


@dataclass
class TerrainMap:
    """Terrain elevation model for ground-height queries and collision detection.

    The terrain is stored as a regular grid of elevations over a rectangular
    region in local (x, y) coordinates.  Queries outside the grid return the
    edge elevation (clamped).

    Attributes:
        elevations: 2-D array of elevations [m], shape (ny, nx).
        origin: (x0, y0) of the grid's south-west corner [m].
        resolution: grid spacing in both x and y [m/cell].
    """
    elevations: np.ndarray = field(
        default_factory=lambda: np.zeros((2, 2))
    )
    origin: Tuple[float, float] = (0.0, 0.0)
    resolution: float = 1.0
    origin_gps: Optional[Tuple[float, float, float]] = None  # (lat, lon, alt)

    # ── Constructors ─────────────────────────────────────────────────────

    @staticmethod
    def flat(elevation: float = 0.0, size: float = 1000.0,
             resolution: float = 10.0) -> 'TerrainMap':
        """Create a flat terrain at a constant elevation.

        Args:
            elevation: Ground height [m].
            size: Extent in x and y [m].
            resolution: Grid cell size [m].
        """
        n = max(int(size / resolution) + 1, 2)
        grid = np.full((n, n), elevation)
        return TerrainMap(
            elevations=grid,
            origin=(-size / 2, -size / 2),
            resolution=resolution,
        )

    @staticmethod
    def from_array(elevations: np.ndarray,
                   origin: Tuple[float, float] = (0.0, 0.0),
                   resolution: float = 1.0) -> 'TerrainMap':
        """Create terrain from a 2-D numpy array of elevations.

        Args:
            elevations: 2-D array, shape (ny, nx).  elevations[0, 0] is at *origin*.
            origin: (x0, y0) of the south-west corner.
            resolution: Metres per grid cell.
        """
        assert elevations.ndim == 2, "elevations must be 2-D"
        return TerrainMap(
            elevations=elevations.astype(float),
            origin=origin,
            resolution=resolution,
        )

    @staticmethod
    def from_stl(path: str,
                 resolution: float = 1.0) -> 'TerrainMap':
        """Load terrain from a binary STL file (e.g. BlenderGIS export).

        The STL is rasterised onto a regular grid.  Each grid cell receives
        the maximum z-value of all triangle vertices that fall within it.

        Args:
            path: Path to binary STL file.
            resolution: Output grid spacing [m].
        """
        vertices = _read_stl_vertices(path)
        if len(vertices) == 0:
            return TerrainMap.flat()

        x_min, y_min = vertices[:, 0].min(), vertices[:, 1].min()
        x_max, y_max = vertices[:, 0].max(), vertices[:, 1].max()

        nx = max(int((x_max - x_min) / resolution) + 1, 2)
        ny = max(int((y_max - y_min) / resolution) + 1, 2)
        grid = np.full((ny, nx), np.nan)

        ix = np.clip(((vertices[:, 0] - x_min) / resolution).astype(int), 0, nx - 1)
        iy = np.clip(((vertices[:, 1] - y_min) / resolution).astype(int), 0, ny - 1)

        for k in range(len(vertices)):
            curr = grid[iy[k], ix[k]]
            z = vertices[k, 2]
            if np.isnan(curr) or z > curr:
                grid[iy[k], ix[k]] = z

        # Fill NaN holes with nearest-neighbor (simple dilation)
        _fill_nan_nearest(grid)

        return TerrainMap(
            elevations=grid,
            origin=(x_min, y_min),
            resolution=resolution,
        )

    @staticmethod
    def from_function(fn, x_range: Tuple[float, float] = (-50, 50),
                      y_range: Tuple[float, float] = (-50, 50),
                      resolution: float = 1.0) -> 'TerrainMap':
        """Create terrain from an analytical function z = fn(x, y).

        Useful for testing with hills, valleys, etc.
        """
        xs = np.arange(x_range[0], x_range[1] + resolution, resolution)
        ys = np.arange(y_range[0], y_range[1] + resolution, resolution)
        xx, yy = np.meshgrid(xs, ys)
        zz = fn(xx, yy)
        return TerrainMap(
            elevations=zz,
            origin=(x_range[0], y_range[0]),
            resolution=resolution,
        )

    @staticmethod
    def from_srtm(lat: float, lon: float, size_km: float = 5.0,
                  resolution: float = 30.0,
                  cache_dir: Optional[str] = None) -> 'TerrainMap':
        """Load terrain from SRTM 1-arc-second .hgt tile.

        Downloads the tile covering (lat, lon) and extracts a region of
        size_km x size_km centered on that point.

        Args:
            lat: Center latitude [degrees].
            lon: Center longitude [degrees].
            size_km: Region size [km].
            resolution: Output grid resolution [m]. SRTM native is ~30m.
            cache_dir: Directory for cached .hgt files. Defaults to /tmp/srtm_cache.
        """
        if cache_dir is None:
            cache_dir = "/tmp/srtm_cache"

        hgt_path = _download_srtm_tile(lat, lon, cache_dir)
        full_grid, tile_lat, tile_lon = _parse_hgt(hgt_path)

        # Extract region around target point
        half_deg = (size_km * 1000.0) / R_EARTH * (180.0 / np.pi)
        lat_min, lat_max = lat - half_deg, lat + half_deg
        lon_min, lon_max = lon - half_deg, lon + half_deg

        # SRTM grid: rows go from N to S, cols go from W to E
        # Row 0 = tile_lat + 1, row 3600 = tile_lat
        nrows, ncols = full_grid.shape
        row_start = max(0, int((tile_lat + 1 - lat_max) * (nrows - 1)))
        row_end = min(nrows, int((tile_lat + 1 - lat_min) * (nrows - 1)) + 1)
        col_start = max(0, int((lon_min - tile_lon) * (ncols - 1)))
        col_end = min(ncols, int((lon_max - tile_lon) * (ncols - 1)) + 1)

        sub_grid = full_grid[row_start:row_end, col_start:col_end].astype(float)

        # Handle void values (-32768)
        sub_grid[sub_grid < -1000] = np.nan
        if np.all(np.isnan(sub_grid)):
            sub_grid = np.zeros_like(sub_grid)
        else:
            _fill_nan_nearest(sub_grid)

        # Resample to target resolution
        srtm_res_m = (1.0 / (nrows - 1)) * (np.pi / 180.0) * R_EARTH * np.cos(np.deg2rad(lat))
        if resolution > srtm_res_m * 1.5:
            step = max(1, int(resolution / srtm_res_m))
            sub_grid = sub_grid[::step, ::step]

        # Convert to local coordinates centered on (lat, lon)
        extent_x = sub_grid.shape[1] * resolution
        extent_y = sub_grid.shape[0] * resolution

        return TerrainMap(
            elevations=sub_grid[::-1],  # flip to S-to-N (increasing Y)
            origin=(-extent_x / 2, -extent_y / 2),
            resolution=resolution,
            origin_gps=(lat, lon, float(np.nanmean(sub_grid))),
        )

    @staticmethod
    def from_hgt(path: str, lat: float, lon: float,
                 size_km: float = 5.0,
                 resolution: float = 30.0) -> 'TerrainMap':
        """Load terrain from a local SRTM .hgt file.

        Args:
            path: Path to .hgt file.
            lat: Center latitude for extraction.
            lon: Center longitude for extraction.
            size_km: Region size [km].
            resolution: Output grid resolution [m].
        """
        full_grid, tile_lat, tile_lon = _parse_hgt(path)

        half_deg = (size_km * 1000.0) / R_EARTH * (180.0 / np.pi)
        lat_min, lat_max = lat - half_deg, lat + half_deg
        lon_min, lon_max = lon - half_deg, lon + half_deg

        nrows, ncols = full_grid.shape
        row_start = max(0, int((tile_lat + 1 - lat_max) * (nrows - 1)))
        row_end = min(nrows, int((tile_lat + 1 - lat_min) * (nrows - 1)) + 1)
        col_start = max(0, int((lon_min - tile_lon) * (ncols - 1)))
        col_end = min(ncols, int((lon_max - tile_lon) * (ncols - 1)) + 1)

        sub_grid = full_grid[row_start:row_end, col_start:col_end].astype(float)
        sub_grid[sub_grid < -1000] = np.nan
        if not np.all(np.isnan(sub_grid)):
            _fill_nan_nearest(sub_grid)
        else:
            sub_grid = np.zeros_like(sub_grid)

        srtm_res_m = (1.0 / (nrows - 1)) * (np.pi / 180.0) * R_EARTH * np.cos(np.deg2rad(lat))
        if resolution > srtm_res_m * 1.5:
            step = max(1, int(resolution / srtm_res_m))
            sub_grid = sub_grid[::step, ::step]

        extent_x = sub_grid.shape[1] * resolution
        extent_y = sub_grid.shape[0] * resolution

        return TerrainMap(
            elevations=sub_grid[::-1],
            origin=(-extent_x / 2, -extent_y / 2),
            resolution=resolution,
            origin_gps=(lat, lon, float(np.nanmean(sub_grid))),
        )

    # ── Queries ──────────────────────────────────────────────────────────

    def get_elevation_gps(self, lat: float, lon: float) -> float:
        """Return ground elevation [m] at GPS coordinates.

        Requires origin_gps to be set (e.g. from from_srtm or from_hgt).
        Converts lat/lon to local (x, y) using the stored GPS origin.
        """
        if self.origin_gps is None:
            raise ValueError("origin_gps not set — use from_srtm() or set origin_gps manually")

        origin_lat, origin_lon, _ = self.origin_gps
        lat_rad = np.deg2rad(lat)
        origin_lat_rad = np.deg2rad(origin_lat)
        origin_lon_rad = np.deg2rad(origin_lon)

        # East-North local frame (Y=North, X=East)
        y = R_EARTH * (lat_rad - origin_lat_rad)
        x = R_EARTH * np.cos(origin_lat_rad) * (np.deg2rad(lon) - origin_lon_rad)

        return self.get_elevation(x, y)

    def get_elevation(self, x: float, y: float) -> float:
        """Return ground elevation [m] at world position (x, y).

        Uses bilinear interpolation.  Coordinates outside the grid are clamped.
        """
        fx = (x - self.origin[0]) / self.resolution
        fy = (y - self.origin[1]) / self.resolution

        ny, nx = self.elevations.shape
        fx = np.clip(fx, 0, nx - 1)
        fy = np.clip(fy, 0, ny - 1)

        ix0 = int(np.floor(fx))
        iy0 = int(np.floor(fy))
        ix1 = min(ix0 + 1, nx - 1)
        iy1 = min(iy0 + 1, ny - 1)

        sx = fx - ix0
        sy = fy - iy0

        z00 = self.elevations[iy0, ix0]
        z10 = self.elevations[iy0, ix1]
        z01 = self.elevations[iy1, ix0]
        z11 = self.elevations[iy1, ix1]

        return float(
            z00 * (1 - sx) * (1 - sy)
            + z10 * sx * (1 - sy)
            + z01 * (1 - sx) * sy
            + z11 * sx * sy
        )

    def check_collision(self, position: np.ndarray) -> bool:
        """Return True if position is at or below the terrain surface."""
        ground_z = self.get_elevation(position[0], position[1])
        return position[2] <= ground_z

    def get_normal(self, x: float, y: float) -> np.ndarray:
        """Approximate surface normal via central differences."""
        h = self.resolution
        dzdx = (self.get_elevation(x + h, y) - self.get_elevation(x - h, y)) / (2 * h)
        dzdy = (self.get_elevation(x, y + h) - self.get_elevation(x, y - h)) / (2 * h)
        n = np.array([-dzdx, -dzdy, 1.0])
        return n / np.linalg.norm(n)

    def export_stl(self, path: str, scale: float = 1.0) -> None:
        """Export terrain as a binary STL mesh.

        Tessellates the elevation grid into triangles (2 per grid cell)
        and writes a binary STL file suitable for Gazebo.

        Args:
            path: Output file path.
            scale: Coordinate scale factor (e.g. to match Gazebo world units).
        """
        ny, nx = self.elevations.shape
        if nx < 2 or ny < 2:
            raise ValueError("Need at least 2x2 grid to export STL")

        n_cells = (nx - 1) * (ny - 1)
        n_triangles = n_cells * 2

        # Build vertex coordinates
        xs = self.origin[0] + np.arange(nx) * self.resolution * scale
        ys = self.origin[1] + np.arange(ny) * self.resolution * scale
        zs = self.elevations * scale

        p = Path(path)
        with open(p, 'wb') as f:
            # 80-byte header
            header = b'Binary STL from TerrainMap.export_stl()' + b'\0' * 41
            f.write(header[:80])
            # Triangle count
            f.write(struct.pack('<I', n_triangles))

            for iy in range(ny - 1):
                for ix in range(nx - 1):
                    # Four corners of this grid cell
                    v00 = (xs[ix],     ys[iy],     zs[iy, ix])
                    v10 = (xs[ix + 1], ys[iy],     zs[iy, ix + 1])
                    v01 = (xs[ix],     ys[iy + 1], zs[iy + 1, ix])
                    v11 = (xs[ix + 1], ys[iy + 1], zs[iy + 1, ix + 1])

                    # Triangle 1: v00, v10, v01
                    n1 = _triangle_normal(v00, v10, v01)
                    f.write(struct.pack('<fff', *n1))
                    f.write(struct.pack('<fff', *v00))
                    f.write(struct.pack('<fff', *v10))
                    f.write(struct.pack('<fff', *v01))
                    f.write(struct.pack('<H', 0))

                    # Triangle 2: v10, v11, v01
                    n2 = _triangle_normal(v10, v11, v01)
                    f.write(struct.pack('<fff', *n2))
                    f.write(struct.pack('<fff', *v10))
                    f.write(struct.pack('<fff', *v11))
                    f.write(struct.pack('<fff', *v01))
                    f.write(struct.pack('<H', 0))

    def export_obj_with_uv(self,
                           path: str,
                           texture_path: Optional[str] = None,
                           material_name: str = "Terrain/Satellite",
                           scale: float = 1.0) -> None:
        """Export terrain as OBJ mesh with UV coordinates and optional texture.

        Args:
            path: Output OBJ path.
            texture_path: Optional texture image path copied next to OBJ.
            material_name: Material name written to MTL.
            scale: Coordinate scale factor.
        """
        ny, nx = self.elevations.shape
        if nx < 2 or ny < 2:
            raise ValueError("Need at least 2x2 grid to export OBJ")

        obj_path = Path(path)
        obj_path.parent.mkdir(parents=True, exist_ok=True)
        mtl_path = obj_path.with_suffix('.mtl')

        xs = self.origin[0] + np.arange(nx) * self.resolution * scale
        ys = self.origin[1] + np.arange(ny) * self.resolution * scale
        zs = self.elevations * scale

        denom_x = max(nx - 1, 1)
        denom_y = max(ny - 1, 1)

        texture_filename = None
        if texture_path is not None:
            src = Path(texture_path)
            if src.exists():
                texture_filename = src.name
                if src.resolve() != (obj_path.parent / src.name).resolve():
                    shutil.copy2(src, obj_path.parent / src.name)

        with open(mtl_path, 'w', encoding='utf-8') as mtl:
            mtl.write(f"newmtl {material_name}\n")
            mtl.write("Ka 1.0 1.0 1.0\n")
            mtl.write("Kd 1.0 1.0 1.0\n")
            mtl.write("Ks 0.0 0.0 0.0\n")
            if texture_filename is not None:
                mtl.write(f"map_Kd {texture_filename}\n")

        with open(obj_path, 'w', encoding='utf-8') as obj:
            obj.write(f"mtllib {mtl_path.name}\n")
            obj.write(f"usemtl {material_name}\n")

            # Vertices in row-major order
            for iy in range(ny):
                for ix in range(nx):
                    obj.write(f"v {xs[ix]:.6f} {ys[iy]:.6f} {zs[iy, ix]:.6f}\n")

            # UVs aligned with grid coordinates
            for iy in range(ny):
                for ix in range(nx):
                    u = ix / denom_x
                    v = iy / denom_y
                    obj.write(f"vt {u:.6f} {v:.6f}\n")

            def idx(x: int, y: int) -> int:
                return y * nx + x + 1

            for iy in range(ny - 1):
                for ix in range(nx - 1):
                    v00 = idx(ix, iy)
                    v10 = idx(ix + 1, iy)
                    v01 = idx(ix, iy + 1)
                    v11 = idx(ix + 1, iy + 1)
                    obj.write(f"f {v00}/{v00} {v10}/{v10} {v01}/{v01}\n")
                    obj.write(f"f {v10}/{v10} {v11}/{v11} {v01}/{v01}\n")

    def export_gazebo_terrain_assets(self,
                                     output_dir: str,
                                     texture_path: Optional[str] = None,
                                     scale: float = 1.0) -> dict:
        """Export terrain mesh/assets for Gazebo with texture fallback metadata.

        Returns dictionary with `obj_path`, `mtl_path`, and `material_name`.
        """
        out_dir = Path(output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        obj_path = out_dir / "terrain.obj"
        material_name = (
            "AntisanaTerrain/SatelliteTextured"
            if texture_path is not None and Path(texture_path).exists()
            else "AntisanaTerrain/HeightColored"
        )
        self.export_obj_with_uv(
            str(obj_path),
            texture_path=texture_path,
            material_name=material_name,
            scale=scale,
        )
        return {
            "obj_path": str(obj_path),
            "mtl_path": str(obj_path.with_suffix('.mtl')),
            "material_name": material_name,
        }

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """Return (x_min, y_min, x_max, y_max) of the terrain grid."""
        ny, nx = self.elevations.shape
        return (
            self.origin[0],
            self.origin[1],
            self.origin[0] + (nx - 1) * self.resolution,
            self.origin[1] + (ny - 1) * self.resolution,
        )

    def is_flat(self, tol: float = 1e-9) -> bool:
        """True when every cell sits at the same elevation (within *tol*)."""
        return bool(np.ptp(self.elevations) <= tol)


# ── STL helpers ──────────────────────────────────────────────────────────────

def _triangle_normal(v0, v1, v2):
    """Compute unit normal of a triangle from three vertices (tuples)."""
    e1 = np.array(v1) - np.array(v0)
    e2 = np.array(v2) - np.array(v0)
    n = np.cross(e1, e2)
    mag = np.linalg.norm(n)
    if mag < 1e-12:
        return (0.0, 0.0, 1.0)
    n = n / mag
    return (float(n[0]), float(n[1]), float(n[2]))


# ── STL reader ───────────────────────────────────────────────────────────────

def _read_stl_vertices(path: str) -> np.ndarray:
    """Read all triangle vertices from a binary STL file.

    Returns an Nx3 array of (x, y, z) vertex coordinates.
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"STL file not found: {path}")

    data = p.read_bytes()
    if len(data) < 84:
        return np.empty((0, 3))

    n_triangles = struct.unpack_from('<I', data, 80)[0]
    expected_size = 84 + n_triangles * 50
    if len(data) < expected_size:
        raise ValueError(
            f"STL file truncated: expected {expected_size} bytes, got {len(data)}"
        )

    vertices = []
    offset = 84
    for _ in range(n_triangles):
        # Skip normal (12 bytes), read 3 vertices (36 bytes), skip attr (2 bytes)
        for v in range(3):
            vx, vy, vz = struct.unpack_from('<fff', data, offset + 12 + v * 12)
            vertices.append((vx, vy, vz))
        offset += 50

    return np.array(vertices) if vertices else np.empty((0, 3))


def _download_srtm_tile(lat: float, lon: float, cache_dir: str) -> str:
    """Download SRTM .hgt tile for given coordinates.

    SRTM tiles are named by their SW corner: e.g. S01W079.hgt
    """
    import os
    import urllib.request

    tile_lat = int(np.floor(lat))
    tile_lon = int(np.floor(lon))

    lat_prefix = "N" if tile_lat >= 0 else "S"
    lon_prefix = "E" if tile_lon >= 0 else "W"
    tile_name = f"{lat_prefix}{abs(tile_lat):02d}{lon_prefix}{abs(tile_lon):03d}.hgt"

    os.makedirs(cache_dir, exist_ok=True)
    local_path = os.path.join(cache_dir, tile_name)

    if os.path.exists(local_path):
        return local_path

    # Try NASA SRTM server (SRTM1 - 1 arc-second)
    base_url = "https://e4ftl01.cr.usgs.gov/MEASURES/SRTMGL1.003/2000.02.11"
    url = f"{base_url}/{tile_name}.zip"

    try:
        zip_path = local_path + ".zip"
        urllib.request.urlretrieve(url, zip_path)
        import zipfile
        with zipfile.ZipFile(zip_path, 'r') as zf:
            zf.extract(tile_name, cache_dir)
        os.remove(zip_path)
    except Exception:
        # Create a synthetic tile for offline/test use
        _create_synthetic_hgt(local_path, tile_lat, tile_lon)

    return local_path


def download_satellite_tile(lat: float,
                            lon: float,
                            cache_dir: str,
                            tile_size: int = 1024) -> Optional[str]:
    """Download or synthesize a satellite texture tile for the given region.

    Uses ESRI World Imagery where available. If download fails, a deterministic
    synthetic texture is generated for offline/test use.
    """
    import os
    import urllib.request

    os.makedirs(cache_dir, exist_ok=True)
    tile_name = f"sat_{lat:+.4f}_{lon:+.4f}_{tile_size}.ppm"
    tile_path = os.path.join(cache_dir, tile_name)
    if os.path.exists(tile_path):
        return tile_path

    # WebMercator tile around (lat, lon)
    try:
        z = 13
        lat_rad = np.deg2rad(lat)
        n = 2 ** z
        xtile = int((lon + 180.0) / 360.0 * n)
        ytile = int((1.0 - np.log(np.tan(lat_rad) + 1.0 / np.cos(lat_rad)) / np.pi) / 2.0 * n)
        url = (
            "https://services.arcgisonline.com/ArcGIS/rest/services/"
            f"World_Imagery/MapServer/tile/{z}/{ytile}/{xtile}"
        )
        png_path = tile_path.replace('.ppm', '.png')
        urllib.request.urlretrieve(url, png_path)
        # Keep function dependency-free: fallback to synthetic if conversion not available
        try:
            from PIL import Image
            Image.open(png_path).convert('RGB').resize((tile_size, tile_size)).save(tile_path)
            os.remove(png_path)
            return tile_path
        except Exception:
            if os.path.exists(png_path):
                os.remove(png_path)
    except Exception:
        pass

    _create_synthetic_satellite_tile(tile_path, tile_size=tile_size)
    return tile_path


def _create_synthetic_hgt(path: str, tile_lat: int, tile_lon: int) -> None:
    """Create a synthetic SRTM tile for testing (no network required).

    Generates elevation data based on approximate real-world elevation
    for well-known locations (e.g. Antisana ~4500m, Quito ~2800m).
    """
    size = 1201  # SRTM3 resolution (3 arc-second)
    base_elevation = 100  # default

    # Known elevations for test regions
    if -1 <= tile_lat <= 0 and -79 <= tile_lon <= -78:
        # Antisana/Quito region
        base_elevation = 4000
        rows = np.linspace(0, 1, size)
        cols = np.linspace(0, 1, size)
        rr, cc = np.meshgrid(rows, cols, indexing='ij')
        # Simulate volcano-like terrain
        dist = np.sqrt((rr - 0.5)**2 + (cc - 0.5)**2)
        grid = base_elevation + 1700 * np.exp(-dist**2 / 0.02)
    else:
        grid = np.full((size, size), base_elevation)

    grid = grid.astype('>i2')
    with open(path, 'wb') as f:
        f.write(grid.tobytes())


def _create_synthetic_satellite_tile(path: str, tile_size: int = 1024) -> None:
    """Create deterministic pseudo-satellite RGB tile in PPM format."""
    xs = np.linspace(0.0, 1.0, tile_size, dtype=float)
    ys = np.linspace(0.0, 1.0, tile_size, dtype=float)
    xx, yy = np.meshgrid(xs, ys)
    ridges = 0.5 + 0.5 * np.sin(16.0 * xx + 12.0 * yy)
    snow = np.clip((yy - 0.65) * 2.4, 0.0, 1.0)
    rock = np.clip(1.0 - np.abs(yy - 0.5) * 2.0, 0.0, 1.0)
    r = np.clip(60 + 100 * rock + 120 * snow, 0, 255).astype(np.uint8)
    g = np.clip(80 + 80 * ridges + 80 * (1.0 - snow), 0, 255).astype(np.uint8)
    b = np.clip(50 + 60 * (1.0 - rock) + 150 * snow, 0, 255).astype(np.uint8)
    rgb = np.stack([r, g, b], axis=2)

    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, 'wb') as f:
        f.write(f"P6\n{tile_size} {tile_size}\n255\n".encode('ascii'))
        f.write(rgb.tobytes())


def _parse_hgt(path: str) -> Tuple[np.ndarray, int, int]:
    """Parse SRTM .hgt file.

    Returns (grid, tile_lat, tile_lon) where grid is (N, N) int16 array.
    SRTM1 = 3601x3601, SRTM3 = 1201x1201.
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"HGT file not found: {path}")

    # Extract tile coordinates from filename
    name = p.stem.upper()
    lat_sign = 1 if name[0] == 'N' else -1
    lon_sign = 1 if name[3] == 'E' else -1
    tile_lat = lat_sign * int(name[1:3])
    tile_lon = lon_sign * int(name[4:7])

    data = p.read_bytes()
    n_samples = len(data) // 2
    size = int(np.sqrt(n_samples))
    grid = np.frombuffer(data, dtype='>i2').reshape((size, size))

    return grid, tile_lat, tile_lon


def _fill_nan_nearest(grid: np.ndarray) -> None:
    """Fill NaN cells in-place with nearest non-NaN value (simple dilation)."""
    mask = np.isnan(grid)
    if not mask.any():
        return

    try:
        from scipy.ndimage import distance_transform_edt
        indices = distance_transform_edt(mask, return_distances=False,
                                          return_indices=True)
        grid[mask] = grid[tuple(indices[:, mask])]
    except ImportError:
        # Stdlib fallback when scipy is absent: pure-numpy nearest-neighbour
        # using the squared-distance argmin over known cells. O(N·M) where
        # M is the count of known cells; fine for terrain grids (≤10⁵ cells)
        # and avoids dragging scipy into the runtime dependency set.
        ys, xs = np.where(~mask)
        if ys.size == 0:
            grid[mask] = 0.0
            return
        nan_ys, nan_xs = np.where(mask)
        # Vectorised pairwise squared distances.
        d2 = (nan_ys[:, None] - ys[None, :]) ** 2 \
            + (nan_xs[:, None] - xs[None, :]) ** 2
        nearest = d2.argmin(axis=1)
        grid[mask] = grid[ys[nearest], xs[nearest]]


# ── Manifest registry ────────────────────────────────────────────────────────
# Phase 3 (terrain integration). Keeps the canonical "what terrains exist
# and how to reproduce them" list in `gazebo/worlds/terrain/manifest.toml`
# so simulator code, Gazebo worlds, and CI parity gates all share one
# definition. Add new function-source terrains by registering them here.

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MANIFEST_PATH = _PROJECT_ROOT / "gazebo" / "worlds" / "terrain" / "manifest.toml"


def _rolling_sine(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Deterministic rolling-hill terrain: 5 m amplitude, 50 m wavelength."""
    return 5.0 * np.sin(x / 50.0) * np.cos(y / 50.0)


_REGISTERED_FNS = {
    "rolling_sine": _rolling_sine,
}


def _read_manifest(manifest_path: Optional[Path]) -> dict:
    import tomllib
    path = Path(manifest_path) if manifest_path else DEFAULT_MANIFEST_PATH
    if not path.is_file():
        raise FileNotFoundError(f"terrain manifest not found: {path}")
    with open(path, "rb") as f:
        return tomllib.load(f)


def _resolve_relative(manifest_path: Optional[Path], rel: str) -> Path:
    base = (Path(manifest_path) if manifest_path
            else DEFAULT_MANIFEST_PATH).parent
    candidate = (base / rel).resolve()
    return candidate


def _require(entry: dict, name: str, key: str):
    """Look up a required manifest field; raise ValueError on miss."""
    if key not in entry:
        raise ValueError(f"terrain '{name}': missing required field '{key}'")
    return entry[key]


def _validate_checksum(path: Path, expected_sha256: Optional[str],
                       label: str) -> None:
    """Validate file SHA-256 when checksum is provided in manifest."""
    if not expected_sha256:
        return
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual.lower() != str(expected_sha256).lower():
        raise ValueError(
            f"terrain '{label}': checksum mismatch for '{path.name}'"
        )


def load_from_manifest(name: str,
                       manifest_path: Optional[Path] = None) -> 'TerrainMap':
    """Build a :class:`TerrainMap` from a manifest entry.

    Args:
        name: Manifest entry name (top-level table in manifest.toml).
        manifest_path: Override the default manifest location.

    Raises:
        FileNotFoundError: manifest file missing.
        ValueError: unknown entry name, missing/unknown source, missing
            required fields, or unknown registered function name.
    """
    manifest = _read_manifest(manifest_path)
    if name not in manifest:
        raise ValueError(
            f"unknown terrain '{name}'; available: {sorted(manifest)}"
        )
    entry = manifest[name]
    source = entry.get("source")
    if source is None:
        raise ValueError(f"terrain '{name}': missing required field 'source'")
    if source == "flat":
        return TerrainMap.flat(
            elevation=float(entry.get("elevation", 0.0)),
            size=float(entry.get("size_m", 1000.0)),
            resolution=float(entry.get("resolution_m", 10.0)),
        )
    if source == "function":
        fn_name = entry.get("fn")
        if fn_name not in _REGISTERED_FNS:
            raise ValueError(
                f"terrain '{name}': unknown fn '{fn_name}'; "
                f"registered: {sorted(_REGISTERED_FNS)}"
            )
        x_range = tuple(float(v) for v in _require(entry, name, "x_range"))
        y_range = tuple(float(v) for v in _require(entry, name, "y_range"))
        return TerrainMap.from_function(
            _REGISTERED_FNS[fn_name],
            x_range=x_range,
            y_range=y_range,
            resolution=float(entry.get("resolution_m", 1.0)),
        )
    if source == "srtm":
        return TerrainMap.from_srtm(
            lat=float(_require(entry, name, "lat")),
            lon=float(_require(entry, name, "lon")),
            size_km=float(entry.get("size_km", 5.0)),
            resolution=float(entry.get("resolution_m", 30.0)),
        )
    if source == "array":
        npz_path = _resolve_relative(manifest_path,
                                     _require(entry, name, "npz_path"))
        _validate_checksum(npz_path, entry.get("checksum_sha256"), name)
        data = np.load(str(npz_path))
        return TerrainMap.from_array(
            data["elevations"],
            origin=tuple(float(v) for v in entry.get("origin", (0.0, 0.0))),
            resolution=float(entry.get("resolution_m", 1.0)),
        )
    if source == "stl":
        stl_path = _resolve_relative(manifest_path,
                                     _require(entry, name, "stl_path"))
        _validate_checksum(stl_path, entry.get("checksum_sha256"), name)
        return TerrainMap.from_stl(
            str(stl_path),
            resolution=float(entry.get("resolution_m", 1.0)),
        )
    raise ValueError(f"terrain '{name}': unknown source '{source}'")


def manifest_entries(manifest_path: Optional[Path] = None) -> list:
    """Return the list of entry names in the manifest (sorted)."""
    return sorted(_read_manifest(manifest_path))
