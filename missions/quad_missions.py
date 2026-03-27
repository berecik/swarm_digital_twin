"""
Quadrotor mission definitions — Valencia et al. (2025) Section 3.2, Table 4.

Carolina Park (Quito): -0.189, -78.488, ~2800m MSL
EPN Campus (Quito):    -0.210, -78.491, ~2820m MSL

Missions are defined as lists of (lat, lon, alt_msl) waypoints.
Altitude values are MSL; the hover experiments use 20m or 30m/40m AGL.
"""

# ── GPS origins ───────────────────────────────────────────────────────

CAROLINA_ORIGIN = {"lat": -0.189, "lon": -78.488, "alt_msl": 2800}
EPN_ORIGIN = {"lat": -0.210, "lon": -78.491, "alt_msl": 2820}

# ── Carolina Park missions ────────────────────────────────────────────

CAROLINA_40 = {
    "name": "carolina_40",
    "description": "Carolina Park hover at 40m AGL (~2840m MSL)",
    "origin": CAROLINA_ORIGIN,
    "altitude_agl": 40.0,
    "waypoints": [
        # Takeoff → hover 40m → small square pattern → land
        (-0.189000, -78.488000, 2840),  # takeoff to 40m AGL
        (-0.188900, -78.487900, 2840),  # NE corner (~11m)
        (-0.188900, -78.488100, 2840),  # NW corner
        (-0.189100, -78.488100, 2840),  # SW corner
        (-0.189100, -78.487900, 2840),  # SE corner
        (-0.189000, -78.488000, 2840),  # return to center
        (-0.189000, -78.488000, 2800),  # land
    ],
}

CAROLINA_20 = {
    "name": "carolina_20",
    "description": "Carolina Park hover at 20m AGL (~2820m MSL)",
    "origin": CAROLINA_ORIGIN,
    "altitude_agl": 20.0,
    "waypoints": [
        (-0.189000, -78.488000, 2820),  # takeoff to 20m AGL
        (-0.188950, -78.487950, 2820),  # NE corner (~6m)
        (-0.188950, -78.488050, 2820),  # NW corner
        (-0.189050, -78.488050, 2820),  # SW corner
        (-0.189050, -78.487950, 2820),  # SE corner
        (-0.189000, -78.488000, 2820),  # return to center
        (-0.189000, -78.488000, 2800),  # land
    ],
}

# ── EPN Campus missions ──────────────────────────────────────────────

EPN_30 = {
    "name": "epn_30",
    "description": "EPN campus hover at 30m AGL (~2850m MSL)",
    "origin": EPN_ORIGIN,
    "altitude_agl": 30.0,
    "waypoints": [
        (-0.210000, -78.491000, 2850),  # takeoff to 30m AGL
        (-0.209900, -78.490900, 2850),  # NE corner (~11m)
        (-0.209900, -78.491100, 2850),  # NW corner
        (-0.210100, -78.491100, 2850),  # SW corner
        (-0.210100, -78.490900, 2850),  # SE corner
        (-0.210000, -78.491000, 2850),  # return to center
        (-0.210000, -78.491000, 2820),  # land
    ],
}

EPN_20 = {
    "name": "epn_20",
    "description": "EPN campus hover at 20m AGL (~2840m MSL)",
    "origin": EPN_ORIGIN,
    "altitude_agl": 20.0,
    "waypoints": [
        (-0.210000, -78.491000, 2840),  # takeoff to 20m AGL
        (-0.209950, -78.490950, 2840),  # NE corner (~6m)
        (-0.209950, -78.491050, 2840),  # NW corner
        (-0.210050, -78.491050, 2840),  # SW corner
        (-0.210050, -78.490950, 2840),  # SE corner
        (-0.210000, -78.491000, 2840),  # return to center
        (-0.210000, -78.491000, 2820),  # land
    ],
}

# ── All missions ─────────────────────────────────────────────────────

ALL_QUAD_MISSIONS = {
    "carolina_40": CAROLINA_40,
    "carolina_20": CAROLINA_20,
    "epn_30": EPN_30,
    "epn_20": EPN_20,
}


def get_mission(name: str) -> dict:
    """Get a quadrotor mission definition by name."""
    if name not in ALL_QUAD_MISSIONS:
        available = ", ".join(sorted(ALL_QUAD_MISSIONS.keys()))
        raise ValueError(f"Unknown mission '{name}'. Available: {available}")
    return ALL_QUAD_MISSIONS[name]


def mission_to_qgc_wpl(mission: dict) -> str:
    """Convert a mission dict to QGC WPL 110 format string."""
    lines = ["QGC WPL 110"]
    wps = mission["waypoints"]

    # Home position
    lines.append(f"0\t1\t0\t16\t0\t0\t0\t0\t{wps[0][0]:.6f}\t{wps[0][1]:.6f}\t{wps[-1][2]:.0f}\t1")

    # Takeoff
    lines.append(f"1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t{mission['altitude_agl']:.0f}\t1")

    # Waypoints
    for i, (lat, lon, alt) in enumerate(wps[1:-1], start=2):
        lines.append(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{lat:.6f}\t{lon:.6f}\t{alt:.0f}\t1")

    # Land
    lat_land, lon_land, alt_land = wps[-1]
    lines.append(f"{len(wps)}\t0\t3\t21\t0\t0\t0\t0\t{lat_land:.6f}\t{lon_land:.6f}\t{alt_land:.0f}\t1")

    return "\n".join(lines) + "\n"
