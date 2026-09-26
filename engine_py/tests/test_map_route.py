"""Map matching and routing on synthetic road networks."""
import heapq

import numpy as np
import pytest

from dhruva.config import load_config
from dhruva.mapmatch import MapMatcher, RoadGraph
from dhruva.mapmatch.synthetic import fork_graph, grid_graph
from dhruva.route import DeviationMonitor, Router

CFG = load_config()


def drive(points, noise=3.5, seed=0, speed=12.0):
    """Turn a list of true ENU points (1 per second) into noisy observations with heading."""
    rng = np.random.default_rng(seed)
    pts = np.asarray(points, float)
    obs = pts + rng.normal(0, noise, pts.shape)
    d = np.gradient(pts, axis=0)
    psi = np.arctan2(d[:, 1], d[:, 0])
    return obs, psi, np.full(len(pts), speed)


def line(a, b, step=12.0):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.hypot(*(b - a)) / step))
    return [a + (b - a) * k / n for k in range(n + 1)]


def run_matcher(graph, obs, psi, speed, cov95=10.0):
    m = MapMatcher(graph, CFG)
    return m, [m.step(float(k), o[0], o[1], cov95, psi[k], speed[k]) for k, o in enumerate(obs)]


# ---- graph & routing primitives ---------------------------------------------------------------
def test_projection_and_metadata():
    g = RoadGraph.from_dict(fork_graph())
    cands = g.candidates(100.0, 4.0, 20.0)
    main = next(c for c in cands if c.edge == "main" and c.direction == +1)
    assert main.s == pytest.approx(100.0, abs=0.5) and main.dist == pytest.approx(4.0, abs=0.5)
    assert main.heading == pytest.approx(0.0, abs=1e-6)
    assert g.edges["down"].tunnel and g.edges["main"].maxspeed_kph == 60.0 and g.edges["up"].name == "Upper Rd"
    rev = next(c for c in cands if c.edge == "main" and c.direction == -1)
    assert abs(rev.heading) == pytest.approx(np.pi, abs=1e-6)


def _dijkstra_all(g, src):
    dist = {src: 0.0}
    heap = [(0.0, src)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, np.inf):
            continue
        for v, w, _, _ in g.adj[u]:
            if d + w < dist.get(v, np.inf):
                dist[v] = d + w
                heapq.heappush(heap, (d + w, v))
    return dist


def test_astar_matches_dijkstra_on_grid():
    g = RoadGraph.from_dict(grid_graph(6, 6, 200.0))
    ref = _dijkstra_all(g, "n0_0")
    for dst in ("n5_5", "n3_2", "n0_4", "n5_0"):
        length, arcs = g.shortest("n0_0", dst)
        assert length == pytest.approx(ref[dst], abs=1e-6)
        assert sum(g.edges[e].length for e, _ in arcs) == pytest.approx(length, abs=1e-6)


def test_route_distance_on_grid_and_direction():
    g = RoadGraph.from_dict(grid_graph(4, 4, 200.0))
    a = next(c for c in g.candidates(50.0, 0.0, 10.0) if c.edge == "h0_0" and c.direction == +1)
    b = next(c for c in g.candidates(350.0, 200.0, 10.0) if c.edge == "h1_1" and c.direction == +1)
    # 150 m to the corner, 200 m north, then 150 m east
    assert g.route_distance(a, b) == pytest.approx(150 + 200 + 150, abs=1.0)
    behind = next(c for c in g.candidates(20.0, 0.0, 10.0) if c.edge == "h0_0" and c.direction == +1)
    assert g.route_distance(a, behind) > 300                      # cannot reverse: must loop round


# ---- map matching --------------------------------------------------------------------------------
def test_match_single_road_position_and_metadata():
    g = RoadGraph.from_dict(fork_graph())
    obs, psi, sp = drive(line((10, 0), (380, 0)))
    _, res = run_matcher(g, obs, psi, sp)
    last = res[-1]
    assert last.edge == "main" and last.direction == +1 and last.confidence > 0.9
    assert abs(last.point[1]) < 1e-6 and last.name == "Main St" and last.maxspeed_kph == 60.0 and not last.tunnel


def test_match_direction_of_travel():
    g = RoadGraph.from_dict(fork_graph())
    obs, psi, sp = drive(line((380, 0), (20, 0)))
    _, res = run_matcher(g, obs, psi, sp)
    assert res[-1].edge == "main" and res[-1].direction == -1


def test_parallel_road_20m_apart_is_disambiguated():
    from dhruva.mapmatch.synthetic import build
    g = RoadGraph.from_dict(build({"a": (0, 0), "b": (600, 0), "c": (0, 25), "d": (600, 25)},
                                  [("r1", "a", "b"), ("r2", "c", "d")]))
    obs, psi, sp = drive(line((10, 0), (590, 0)), seed=3)
    _, res = run_matcher(g, obs, psi, sp)
    correct = np.mean([r.edge == "r1" for r in res[10:]])
    assert correct >= 0.9


def test_fork_chooses_the_right_branch_and_lag_agrees():
    g = RoadGraph.from_dict(fork_graph(main_len=400, branch_len=500, spread_deg=12))
    for branch, sign in (("up", +1), ("down", -1)):
        a = np.radians(12.0)
        end = (400 + 480 * np.cos(a), sign * 480 * np.sin(a))
        pts = line((10, 0), (400, 0)) + line((400, 0), end)[1:]
        obs, psi, sp = drive(pts, seed=5)
        m, res = run_matcher(g, obs, psi, sp)
        assert res[-1].edge == branch and res[-1].confidence > 0.7
        assert m.lagged().edge == branch
        assert (res[-1].tunnel) == (branch == "down")


def test_lagged_match_is_five_seconds_behind():
    g = RoadGraph.from_dict(fork_graph())
    obs, psi, sp = drive(line((10, 0), (390, 0), step=10.0))
    m, res = run_matcher(g, obs, psi, sp)
    assert m.lagged().t == pytest.approx(res[-1].t - 5.0)


def test_gap_off_the_network_resets_and_rematches():
    g = RoadGraph.from_dict(fork_graph())
    m = MapMatcher(g, CFG)
    assert m.step(0.0, 100.0, 2.0, 5.0, 0.0, 12.0) is not None
    assert m.step(1.0, 100.0, 900.0, 5.0, 0.0, 12.0) is None       # nowhere near a road
    again = m.step(2.0, 150.0, 1.0, 5.0, 0.0, 12.0)
    assert again is not None and again.edge == "main"


# ---- routing ---------------------------------------------------------------------------------------
def test_route_from_match_covers_remaining_edge_and_ends_at_destination():
    g = RoadGraph.from_dict(grid_graph(6, 6, 200.0))
    r = Router(g, CFG)
    m = MapMatcher(g, CFG)
    match = m.step(0.0, 60.0, 1.0, 5.0, 0.0, 12.0)
    route = r.plan_from_match(match, "n5_3")
    assert route.arcs[0][0] == "h0_0" and route.arcs[-1][0] in ("h4_3", "v5_2", "v5_3")
    assert route.length_m == pytest.approx(1000 + 600 - 60, abs=5.0)
    assert np.allclose(route.xy[0], match.point, atol=0.5)


def test_no_reroute_while_following_the_route():
    g = RoadGraph.from_dict(grid_graph(6, 6, 200.0))
    r = Router(g, CFG)
    route = r.plan_from_node("n0_0", "n5_0")
    mon = DeviationMonitor(r, "n5_0", route)
    m = MapMatcher(g, CFG)
    for k, p in enumerate(line((10, 0), (990, 0), step=15.0)):
        match = m.step(float(k), p[0], p[1] + 2.0, 5.0, 0.0, 12.0)
        assert mon.update(float(k), match, p[0], p[1]) is None
    assert mon.reroutes == []


def test_reroute_after_wrong_turn_is_fast_and_valid():
    g = RoadGraph.from_dict(grid_graph(6, 6, 200.0))
    r = Router(g, CFG)
    route = r.plan_from_node("n0_0", "n5_0")                      # straight east along row 0
    mon = DeviationMonitor(r, "n5_0", route)
    m = MapMatcher(g, CFG)
    pts = line((10, 0), (400, 0), step=15.0) + line((400, 0), (400, 400), step=15.0)[1:]   # turns north at n2_0
    new = None
    for k, p in enumerate(pts):
        match = m.step(float(k), p[0], p[1], 5.0, 0.0 if k < 27 else np.pi / 2, 12.0)
        new = mon.update(float(k), match, p[0], p[1]) or new
    assert new is not None and len(mon.reroutes) >= 1
    assert new.dest == "n5_0" and new.edge_ids != route.edge_ids
    assert max(mon.reroute_ms) < 50.0                             # small graph; timed for the record
    assert new.arcs[0][0].startswith("v2_")                       # continues along the road it is actually on


def test_cross_track_deviation_needs_more_than_three_seconds():
    g = RoadGraph.from_dict(grid_graph(4, 4, 200.0))
    r = Router(g, CFG)
    route = r.plan_from_node("n0_0", "n3_0")
    mon = DeviationMonitor(r, "n3_0", route)

    class FakeMatch:                                              # matched edge stays on the route
        edge, direction, s = "h1_0", +1, 20.0

    assert mon.update(0.0, FakeMatch, 300.0, 45.0) is None        # 45 m off: timer starts
    assert mon.update(2.5, FakeMatch, 320.0, 45.0) is None        # not yet 3 s
    assert mon.update(3.6, FakeMatch, 340.0, 45.0) is not None    # > 3 s off
