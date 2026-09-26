"""Road graph in local ENU metres, loaded from maps/road_graph.json.

JSON: {"origin": [lat0, lon0],
       "nodes": {"n1": [lat, lon], ...},
       "edges": [{"id": "e1", "u": "n1", "v": "n2", "geometry": [[lat, lon], ...],   # optional, u -> v
                  "name": "...", "tunnel": false, "maxspeed_kph": 50, "oneway": false}, ...]}
A candidate is (edge_id, direction): direction +1 travels u -> v (increasing along-edge distance s), -1 travels v -> u.
"""
import heapq
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..geo import to_enu, to_latlon


@dataclass
class Edge:
    id: str
    u: str
    v: str
    xy: np.ndarray        # (k, 2) ENU polyline u -> v
    cum: np.ndarray       # (k,) cumulative length
    name: str
    tunnel: bool
    maxspeed_kph: float
    oneway: bool

    @property
    def length(self):
        return float(self.cum[-1])


@dataclass
class Candidate:
    edge: str
    direction: int
    s: float              # along-edge distance from u [m]
    dist: float           # perpendicular distance from the query point [m]
    heading: float        # travel heading psi (rad, CCW from East) at the projection, in the travel direction
    point: tuple          # projected ENU point


class RoadGraph:
    def __init__(self, origin, nodes_ll, edges):
        self.origin = tuple(origin)
        self.nodes = {k: np.array(to_enu(lat, lon, *self.origin)) for k, (lat, lon) in nodes_ll.items()}
        self.edges = {e.id: e for e in edges}
        self.adj = {k: [] for k in self.nodes}     # node -> [(neighbour, length, edge_id, direction)]
        for e in edges:
            self.adj[e.u].append((e.v, e.length, e.id, +1))
            if not e.oneway:
                self.adj[e.v].append((e.u, e.length, e.id, -1))
        seg_a, seg_b, seg_edge, seg_off = [], [], [], []
        for e in edges:
            for i in range(len(e.xy) - 1):
                seg_a.append(e.xy[i])
                seg_b.append(e.xy[i + 1])
                seg_edge.append(e.id)
                seg_off.append(e.cum[i])
        self._a, self._b = np.array(seg_a), np.array(seg_b)
        self._edge, self._off = np.array(seg_edge), np.array(seg_off)
        self._ab = self._b - self._a
        self._len2 = np.maximum((self._ab ** 2).sum(1), 1e-12)

    # ---- construction -----------------------------------------------------------------
    @classmethod
    def from_json(cls, path):
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(d)

    @classmethod
    def from_dict(cls, d):
        origin = d["origin"]
        nodes = d["nodes"]
        edges = []
        for e in d["edges"]:
            pts = e.get("geometry") or [nodes[e["u"]], nodes[e["v"]]]
            xy = np.array([to_enu(lat, lon, *origin) for lat, lon in pts]).reshape(-1, 2)
            cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
            edges.append(Edge(e["id"], e["u"], e["v"], xy, cum, e.get("name", ""), bool(e.get("tunnel", False)),
                              float(e.get("maxspeed_kph", 50.0)), bool(e.get("oneway", False))))
        return cls(origin, nodes, edges)

    # ---- geometry ---------------------------------------------------------------------
    def latlon(self, e, n):
        return to_latlon(e, n, *self.origin)

    def enu(self, lat, lon):
        return tuple(float(v) for v in to_enu(lat, lon, *self.origin))

    def _project_all(self, p):
        t = np.clip(((p - self._a) * self._ab).sum(1) / self._len2, 0.0, 1.0)
        proj = self._a + t[:, None] * self._ab
        return proj, t, np.hypot(*(proj - p).T)

    def candidates(self, e, n, radius):
        """Best projection per (edge, direction) within `radius` metres of (e, n)."""
        p = np.array([e, n])
        proj, t, dist = self._project_all(p)
        idx = np.flatnonzero(dist <= radius)
        best = {}
        for i in idx:
            eid = self._edge[i]
            if eid not in best or dist[i] < best[eid][0]:
                best[eid] = (dist[i], i)
        out = []
        for eid, (d, i) in best.items():
            s = float(self._off[i] + t[i] * np.sqrt(self._len2[i]))
            psi = float(np.arctan2(self._ab[i][1], self._ab[i][0]))
            edge = self.edges[eid]
            out.append(Candidate(eid, +1, s, float(d), psi, tuple(proj[i])))
            if not edge.oneway:
                out.append(Candidate(eid, -1, s, float(d), float((psi + np.pi + np.pi) % (2 * np.pi) - np.pi),
                                     tuple(proj[i])))
        return out

    def point_at(self, edge_id, s):
        e = self.edges[edge_id]
        s = float(np.clip(s, 0.0, e.length))
        return float(np.interp(s, e.cum, e.xy[:, 0])), float(np.interp(s, e.cum, e.xy[:, 1]))

    # ---- shortest paths ----------------------------------------------------------------------
    def shortest(self, src, dst, cutoff=None):
        """A* over nodes (straight-line heuristic). Returns (length, [(edge_id, direction), ...]) or (inf, [])."""
        if src == dst:
            return 0.0, []
        goal = self.nodes[dst]
        best = {src: 0.0}
        prev = {}
        heap = [(float(np.hypot(*(self.nodes[src] - goal))), 0.0, src)]
        while heap:
            _, g, u = heapq.heappop(heap)
            if u == dst:
                path, node = [], dst
                while node != src:
                    node, eid, d = prev[node]
                    path.append((eid, d))
                return g, path[::-1]
            if g > best.get(u, np.inf):
                continue
            for v, length, eid, d in self.adj[u]:
                ng = g + length
                if cutoff is not None and ng > cutoff:
                    continue
                if ng < best.get(v, np.inf):
                    best[v] = ng
                    prev[v] = (u, eid, d)
                    heapq.heappush(heap, (ng + float(np.hypot(*(self.nodes[v] - goal))), ng, v))
        return float("inf"), []

    def exit_node(self, edge_id, direction):
        e = self.edges[edge_id]
        return e.v if direction == +1 else e.u

    def entry_node(self, edge_id, direction):
        e = self.edges[edge_id]
        return e.u if direction == +1 else e.v

    def route_distance(self, a, b, cutoff=400.0):
        """Distance a vehicle travels from candidate a to candidate b along the network (inf if unreachable)."""
        if a.edge == b.edge and a.direction == b.direction:
            d = (b.s - a.s) * a.direction
            if d >= -3.0:                      # allow a little backwards slop from noise
                return max(d, 0.0)
        ea = self.edges[a.edge]
        eb = self.edges[b.edge]
        exit_d = (ea.length - a.s) if a.direction == +1 else a.s
        entry_d = b.s if b.direction == +1 else eb.length - b.s
        length, _ = self.shortest(self.exit_node(a.edge, a.direction), self.entry_node(b.edge, b.direction),
                                  cutoff=cutoff)
        return exit_d + length + entry_d
