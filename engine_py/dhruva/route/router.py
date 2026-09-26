"""A* routing and deviation-triggered rerouting on the road graph. Master spec section 7.5."""
import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Route:
    dest: str
    arcs: list                    # [(edge_id, direction), ...] in travel order
    length_m: float
    xy: np.ndarray                # (k, 2) ENU polyline of the whole route
    edge_ids: set = field(default_factory=set)


class Router:
    def __init__(self, graph, cfg):
        self.g = graph
        self.c = cfg["route"]

    def _polyline(self, arcs, first_partial=None):
        pts = []
        for i, (eid, d) in enumerate(arcs):
            xy = self.g.edges[eid].xy if d == +1 else self.g.edges[eid].xy[::-1]
            if i == 0 and first_partial is not None:
                s = first_partial.s
                px, py = self.g.point_at(eid, s)
                if d == +1:
                    cut = np.searchsorted(self.g.edges[eid].cum, s, side="right")
                    xy = np.vstack([[px, py], self.g.edges[eid].xy[cut:]])
                else:
                    cut = np.searchsorted(self.g.edges[eid].cum, s, side="left")
                    xy = np.vstack([[px, py], self.g.edges[eid].xy[:cut][::-1]])
            pts.append(xy)
        return np.vstack(pts) if pts else np.zeros((0, 2))

    def plan_from_node(self, src, dest):
        length, arcs = self.g.shortest(src, dest)
        if not np.isfinite(length):
            return None
        return Route(dest, arcs, float(length), self._polyline(arcs), {a[0] for a in arcs})

    def plan_from_match(self, match, dest):
        """Route from the current position: finish the matched edge, then shortest path from its exit node."""
        exit_node = self.g.exit_node(match.edge, match.direction)
        length, arcs = self.g.shortest(exit_node, dest)
        if not np.isfinite(length):
            return None
        edge = self.g.edges[match.edge]
        rest = (edge.length - match.s) if match.direction == +1 else match.s
        all_arcs = [(match.edge, match.direction)] + arcs
        return Route(dest, all_arcs, float(length + rest), self._polyline(all_arcs, first_partial=match),
                     {a[0] for a in all_arcs})

    @staticmethod
    def cross_track_m(route, e, n):
        """Distance from (e, n) to the route polyline."""
        p = np.array([e, n])
        a, b = route.xy[:-1], route.xy[1:]
        ab = b - a
        t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab ** 2).sum(1), 1e-12), 0, 1)
        return float(np.min(np.hypot(*(a + t[:, None] * ab - p).T)))


class DeviationMonitor:
    """Reroutes when the vehicle is more than 30 m off the route for more than 3 s, or the matched edge is not on it."""

    def __init__(self, router, dest, route=None):
        self.router = router
        self.dest = dest
        self.route = route
        self._off_since = None
        self.reroute_ms = []
        self.reroutes = []            # (t, reason)

    def update(self, t, match, e, n):
        """Feed the latest map match and filter position. Returns a new Route if a reroute happened, else None."""
        if self.route is None or match is None:
            return None
        c = self.router.c
        far = Router.cross_track_m(self.route, e, n) > c["deviation_cross_track_m"]
        off_route = match.edge not in self.route.edge_ids
        if far or off_route:
            if self._off_since is None:
                self._off_since = t
            timed_out = t - self._off_since > c["deviation_duration_s"]
            if not (timed_out or off_route):
                return None
        else:
            self._off_since = None
            return None
        t0 = time.perf_counter()
        new = self.router.plan_from_match(match, self.dest)
        self.reroute_ms.append((time.perf_counter() - t0) * 1000.0)
        self.reroutes.append((t, "off route" if off_route else "cross-track"))
        self._off_since = None
        if new is not None:
            self.route = new
        return new
