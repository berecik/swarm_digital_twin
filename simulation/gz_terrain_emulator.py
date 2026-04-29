"""
Gazebo-equivalent heightmap interpolation.

A pure-Python port of the algorithm Gazebo's
``gz::physics::HeightmapShape::HeightAt`` runs against the STL/heightmap
mesh that ships from `terrain.export_stl()`. Lets CI assert that
`terrain.get_elevation(x, y)` matches what Gazebo would return at the
same query point — without spinning up a Gazebo container.

The contract we mimic (per the Gazebo source and the SDF reference):

* The heightmap is a regular grid of `(nx, ny)` cells with spacing
  `(sx, sy)` over an axis-aligned rectangle.
* `HeightAt(x, y)` clamps the query inside the grid bounds, finds the
  enclosing cell, and returns the bilinear interpolation of the four
  corner heights.

`TerrainMap.get_elevation` already does the same bilinear interpolation
(see `terrain.py::TerrainMap.get_elevation`). The emulator below
re-derives the value from the grid directly so the parity test catches
any regression in either side.
"""

from __future__ import annotations

import numpy as np


def gz_height_at(elevations: np.ndarray, origin: tuple,
                 resolution: float, x: float, y: float) -> float:
    """Return the bilinear heightmap height at world (x, y).

    Args:
        elevations: 2-D grid, shape (ny, nx). ``elevations[0, 0]`` is at
            the heightmap's south-west corner.
        origin: (x_sw, y_sw) world coordinates of the SW corner [m].
        resolution: cell spacing [m] (Gazebo heightmaps are square).
        x, y: query world coordinates [m].

    Returns:
        Interpolated elevation [m].
    """
    ny, nx = elevations.shape
    fx = (x - origin[0]) / resolution
    fy = (y - origin[1]) / resolution
    # Gazebo clamps queries outside the heightmap to the edge.
    fx = float(np.clip(fx, 0.0, nx - 1))
    fy = float(np.clip(fy, 0.0, ny - 1))
    ix0 = int(np.floor(fx))
    iy0 = int(np.floor(fy))
    ix1 = min(ix0 + 1, nx - 1)
    iy1 = min(iy0 + 1, ny - 1)
    dx = fx - ix0
    dy = fy - iy0
    z00 = float(elevations[iy0, ix0])
    z10 = float(elevations[iy0, ix1])
    z01 = float(elevations[iy1, ix0])
    z11 = float(elevations[iy1, ix1])
    # Standard 2-D bilinear interpolation.
    z0 = z00 * (1 - dx) + z10 * dx
    z1 = z01 * (1 - dx) + z11 * dx
    return float(z0 * (1 - dy) + z1 * dy)


def parity_samples(terrain, n: int = 100, seed: int = 7) -> tuple:
    """Return ``(deltas, max_delta, rmse)`` between TerrainMap and the
    Gazebo emulator over ``n`` deterministic interior samples.

    Used by the parity test — extracted as a helper so the same logic
    can be invoked from a future nightly Gazebo lane (replacing the
    emulator with the live Gazebo query).
    """
    x_min, y_min, x_max, y_max = terrain.bounds
    margin_x = 0.05 * (x_max - x_min)
    margin_y = 0.05 * (y_max - y_min)
    rng = np.random.default_rng(seed)
    xs = rng.uniform(x_min + margin_x, x_max - margin_x, n)
    ys = rng.uniform(y_min + margin_y, y_max - margin_y, n)
    deltas = np.array([
        abs(
            terrain.get_elevation(float(x), float(y))
            - gz_height_at(terrain.elevations, terrain.origin,
                           terrain.resolution, float(x), float(y))
        )
        for x, y in zip(xs, ys)
    ])
    return deltas, float(deltas.max()), float(np.sqrt(np.mean(deltas ** 2)))
