import numpy as np
import pytest

from dhruva.geo import heading_deg_to_psi, offset_latlon, psi_to_heading_deg, to_enu, to_latlon, wrap_pi

LAT0, LON0 = 28.6139, 77.2090


def test_round_trip_under_1mm():
    rng = np.random.default_rng(0)
    e = rng.uniform(-10_000, 10_000, 1000)
    n = rng.uniform(-10_000, 10_000, 1000)
    lat, lon = to_latlon(e, n, LAT0, LON0)
    e2, n2 = to_enu(lat, lon, LAT0, LON0)
    assert np.max(np.hypot(e2 - e, n2 - n)) < 1e-3


def test_origin_maps_to_zero():
    e, n = to_enu(LAT0, LON0, LAT0, LON0)
    assert e == 0 and n == 0


@pytest.mark.parametrize("psi,heading", [
    (0.0, 90.0),              # East
    (np.pi / 2, 0.0),         # North
    (np.pi, 270.0),           # West
    (-np.pi / 2, 180.0),      # South
    (np.pi / 4, 45.0),        # North-East
])
def test_psi_to_heading_table(psi, heading):
    assert psi_to_heading_deg(psi) == pytest.approx(heading, abs=1e-9)
    assert heading_deg_to_psi(heading) == pytest.approx(wrap_pi(psi), abs=1e-9)


@pytest.mark.parametrize("a,expected", [
    (3 * np.pi, np.pi), (-np.pi, np.pi), (np.pi, np.pi), (0.1 + 2 * np.pi, 0.1), (np.pi + 0.1, -np.pi + 0.1),
])
def test_wrap_pi_range_is_half_open(a, expected):
    assert wrap_pi(a) == pytest.approx(expected, abs=1e-9)


def test_offset_bearing_is_clockwise_from_north():
    lat, lon = offset_latlon(LAT0, LON0, 100.0, 90.0)   # due east
    e, n = to_enu(lat, lon, LAT0, LON0)
    assert e == pytest.approx(100.0, abs=0.01) and n == pytest.approx(0.0, abs=0.01)
    lat, lon = offset_latlon(LAT0, LON0, 100.0, 0.0)    # due north
    e, n = to_enu(lat, lon, LAT0, LON0)
    assert e == pytest.approx(0.0, abs=0.01) and n == pytest.approx(100.0, abs=0.01)
