"""Local ENU <-> lat/lon helpers (equirectangular, fine for trips of ~10 km)."""
import numpy as np

EARTH_R = 6371008.8  # metres, mean radius


def to_enu(lat, lon, lat0, lon0):
    lat, lon = np.asarray(lat, float), np.asarray(lon, float)
    n = np.radians(lat - lat0) * EARTH_R
    e = np.radians(lon - lon0) * EARTH_R * np.cos(np.radians(lat0))
    return e, n


def to_latlon(e, n, lat0, lon0):
    e, n = np.asarray(e, float), np.asarray(n, float)
    lat = lat0 + np.degrees(n / EARTH_R)
    lon = lon0 + np.degrees(e / (EARTH_R * np.cos(np.radians(lat0))))
    return lat, lon


def offset_latlon(lat, lon, dist_m, bearing_deg):
    """Move lat/lon by dist_m along bearing_deg (clockwise from north)."""
    b = np.radians(bearing_deg)
    lat = np.asarray(lat, float)
    lon = np.asarray(lon, float)
    dlat = np.degrees(dist_m * np.cos(b) / EARTH_R)
    dlon = np.degrees(dist_m * np.sin(b) / (EARTH_R * np.cos(np.radians(lat))))
    return lat + dlat, lon + dlon


def wrap_pi(a):
    """Wrap angle(s) in radians to (-pi, pi]."""
    w = (np.asarray(a, float) + np.pi) % (2 * np.pi) - np.pi
    return np.where(w == -np.pi, np.pi, w)


def psi_to_heading_deg(psi):
    """Internal yaw psi (rad, CCW from East) -> UI heading (deg, clockwise from North)."""
    return (90.0 - np.degrees(np.asarray(psi, float))) % 360.0


def heading_deg_to_psi(heading_deg):
    return wrap_pi(np.radians(90.0 - np.asarray(heading_deg, float)))
