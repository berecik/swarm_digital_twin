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

    # ── Queries ──────────────────────────────────────────────────────────

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
