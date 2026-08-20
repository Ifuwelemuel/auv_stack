"""Local ENU frame: lat/lon <-> metres about a fixed origin (ADR-028).
"""
import math

R_EARTH = 6_371_000.0   # mean Earth radius [m]


class LocalFrame:
    """Fix an origin at the first good fix; convert every later fix to
    east/north metres. x = east, y = north (ENU, REP-103)."""

    def __init__(self, lat0_deg: float, lon0_deg: float):
        self._lat0 = math.radians(lat0_deg)
        self._lon0 = math.radians(lon0_deg)
        # cos(lat0): metres-per-degree of longitude shrinks with latitude.
        # Frozen at the origin — the flat-earth approximation in one number.
        self._coslat0 = math.cos(self._lat0)

    def to_local(self, lat_deg: float, lon_deg: float):
        """(lat, lon) -> (east_m, north_m) relative to the origin."""
        dlat = math.radians(lat_deg) - self._lat0
        dlon = math.radians(lon_deg) - self._lon0
        east = R_EARTH * dlon * self._coslat0
        north = R_EARTH * dlat
        return east, north

    def to_global(self, east_m: float, north_m: float):
        """(east_m, north_m) -> (lat, lon). Inverse, for logging/display."""
        lat = self._lat0 + north_m / R_EARTH
        lon = self._lon0 + east_m / (R_EARTH * self._coslat0)
        return math.degrees(lat), math.degrees(lon)