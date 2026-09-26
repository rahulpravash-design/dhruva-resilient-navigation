"""Small synthetic road networks for tests and demos (lat/lon JSON dicts, see graph.py)."""
import numpy as np

from ..geo import to_latlon

ORIGIN = (28.6139, 77.2090)


def _ll(e, n):
    lat, lon = to_latlon(e, n, *ORIGIN)
    return [float(lat), float(lon)]


def build(nodes_enu, edges, origin=ORIGIN):
    """nodes_enu: {id: (e, n)}; edges: [(id, u, v, {optional attrs}, [optional ENU polyline])]."""
    d = {"origin": list(origin), "nodes": {k: _ll(*p) for k, p in nodes_enu.items()}, "edges": []}
    for item in edges:
        eid, u, v = item[:3]
        attrs = item[3] if len(item) > 3 else {}
        poly = item[4] if len(item) > 4 else None
        e = {"id": eid, "u": u, "v": v, **attrs}
        if poly is not None:
            e["geometry"] = [_ll(*p) for p in poly]
        d["edges"].append(e)
    return d


def grid_graph(nx=6, ny=6, spacing=200.0):
    """Two-way grid of roads; node ids 'n{i}_{j}', edge ids 'h{i}_{j}' (east) and 'v{i}_{j}' (north)."""
    nodes = {f"n{i}_{j}": (i * spacing, j * spacing) for i in range(nx) for j in range(ny)}
    edges = []
    for i in range(nx):
        for j in range(ny):
            if i + 1 < nx:
                edges.append((f"h{i}_{j}", f"n{i}_{j}", f"n{i + 1}_{j}", {"name": f"Row {j}", "maxspeed_kph": 50}))
            if j + 1 < ny:
                edges.append((f"v{i}_{j}", f"n{i}_{j}", f"n{i}_{j + 1}", {"name": f"Col {i}", "maxspeed_kph": 50}))
    return build(nodes, edges)


def fork_graph(main_len=400.0, branch_len=500.0, spread_deg=12.0):
    """A road running east that forks into two branches at +/- spread_deg. Edges: main, up, down."""
    a = np.radians(spread_deg)
    fork = (main_len, 0.0)
    nodes = {"start": (0.0, 0.0), "fork": fork,
             "up_end": (main_len + branch_len * np.cos(a), branch_len * np.sin(a)),
             "down_end": (main_len + branch_len * np.cos(a), -branch_len * np.sin(a))}
    edges = [("main", "start", "fork", {"name": "Main St", "maxspeed_kph": 60}),
             ("up", "fork", "up_end", {"name": "Upper Rd", "maxspeed_kph": 40}),
             ("down", "fork", "down_end", {"name": "Lower Rd", "maxspeed_kph": 40, "tunnel": True})]
    return build(nodes, edges)
