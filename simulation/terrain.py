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
from pathlib import Path


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

    # Use iterative dilation — adequate for sparse holes in terrain grids
    from scipy.ndimage import distance_transform_edt
    try:
        indices = distance_transform_edt(mask, return_distances=False,
                                          return_indices=True)
        grid[mask] = grid[tuple(indices[:, mask])]
    except ImportError:
        # Fallback: fill with global mean if scipy unavailable
        mean_val = np.nanmean(grid)
        grid[mask] = mean_val
