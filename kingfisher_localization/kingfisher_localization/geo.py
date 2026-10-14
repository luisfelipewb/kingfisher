# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Geodetic (lat/lon/alt) -> local ENU, and the site files.

``PureENU`` is copied from ``lac_symphonie/geo.py`` (tested there against gz-sim to < 1 cm),
so that ``map`` on the boat is the same tangent plane as the Gazebo world frame. Not UTM:
at the lake, UTM grid north is ~2 deg off true north.
"""

import math

import yaml

# WGS-84 ellipsoid
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def _geodetic_to_ecef(lat_deg, lon_deg, alt):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    n = _A / math.sqrt(1.0 - _E2 * sl * sl)
    return ((n + alt) * cl * math.cos(lon),
            (n + alt) * cl * math.sin(lon),
            (n * (1.0 - _E2) + alt) * sl)


class PureENU:
    """WGS-84 geodetic -> local ENU tangent plane, no external dependency."""

    def __init__(self, lat0, lon0, alt0):
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        self._o = _geodetic_to_ecef(lat0, lon0, alt0)
        la, lo = math.radians(lat0), math.radians(lon0)
        sa, ca, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
        # rows: east, north, up
        self._r = ((-so, co, 0.0),
                   (-sa * co, -sa * so, ca),
                   (ca * co, ca * so, sa))

    def local_from_latlon(self, lat, lon, alt=None):
        alt = self.alt0 if alt is None else alt
        d = [a - b for a, b in zip(_geodetic_to_ecef(lat, lon, alt), self._o)]
        return tuple(sum(r[i] * d[i] for i in range(3)) for r in self._r)


def load_site(path):
    """Return (PureENU at the site's datum, the site's points) from a config/sites file."""
    with open(path) as f:
        site = yaml.safe_load(f)
    d = site['datum']
    return PureENU(d['latitude'], d['longitude'], d['altitude']), site.get('points', [])
