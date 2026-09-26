"""Online HMM / Viterbi map matching with a fixed lag. Master spec section 7.5.

States are (edge, direction) candidates within max(50 m, 3 sigma) of the filter position. Emission is Gaussian in
the perpendicular distance; the transition compares the network distance between candidates with the filter's own
displacement (beta = 8 m) and a heading term (20 deg). `step` returns the filtered best match now; `lagged` returns the
smoothed match `lag_s` seconds ago.
"""
from collections import deque
from dataclasses import dataclass

import numpy as np

CHI2_95_2DOF = 5.991
NO_ROUTE = -1e9


@dataclass
class Match:
    t: float
    edge: str
    direction: int
    s: float
    point: tuple            # projected ENU point
    dist: float             # distance from the filter position to the road [m]
    confidence: float       # posterior of this candidate among the current candidates
    name: str
    tunnel: bool
    maxspeed_kph: float


class MapMatcher:
    def __init__(self, graph, cfg, step_s=1.0):
        self.g = graph
        self.c = cfg["mapmatch"]
        self.lag_steps = max(1, round(self.c["lag_s"] / step_s))
        self.steps = deque(maxlen=self.lag_steps + 1)   # each: dict(t, e, n, cands, logp, back)

    def reset(self):
        self.steps.clear()

    def _match(self, step, idx, conf):
        c = step["cands"][idx]
        edge = self.g.edges[c.edge]
        return Match(step["t"], c.edge, c.direction, c.s, c.point, c.dist, float(conf), edge.name, edge.tunnel,
                     edge.maxspeed_kph)

    def step(self, t, e, n, cov95_m, psi, speed):
        """Advance the HMM by one observation of the filter position. Returns the filtered best Match or None."""
        c = self.c
        sigma = max(c["emission_sigma_min_m"], cov95_m / np.sqrt(CHI2_95_2DOF))
        radius = max(c["candidate_radius_min_m"], c["candidate_radius_sigma_mult"] * sigma)
        cands = self.g.candidates(e, n, radius)
        if not cands:
            self.reset()
            return None
        logp = -np.array([k.dist for k in cands]) ** 2 / (2 * sigma ** 2)
        if speed > 2.0:                                               # heading is meaningless when stopped
            dpsi = np.array([(k.heading - psi + np.pi) % (2 * np.pi) - np.pi for k in cands])
            logp = logp - dpsi ** 2 / (2 * np.radians(c["heading_sigma_deg"]) ** 2)
        back = np.full(len(cands), -1, dtype=int)
        if self.steps:
            prev = self.steps[-1]
            d_dr = float(np.hypot(e - prev["e"], n - prev["n"]))
            new = np.full(len(cands), -np.inf)
            for j, b in enumerate(cands):
                scores = np.full(len(prev["cands"]), NO_ROUTE)
                for i, a in enumerate(prev["cands"]):
                    d_route = self.g.route_distance(a, b)
                    if np.isfinite(d_route):
                        scores[i] = prev["logp"][i] - abs(d_route - d_dr) / c["transition_beta_m"]
                    else:
                        scores[i] = prev["logp"][i] + NO_ROUTE
                back[j] = int(np.argmax(scores))
                new[j] = scores[back[j]] + logp[j]
            logp = new
        logp = logp - logp.max()
        self.steps.append({"t": t, "e": e, "n": n, "cands": cands, "logp": logp, "back": back})
        post = np.exp(logp)
        post /= post.sum()
        best = int(np.argmax(logp))
        return self._match(self.steps[-1], best, post[best])

    def lagged(self):
        """Smoothed match for the observation `lag_s` before the latest one (None until enough history)."""
        if len(self.steps) < 2:
            return None
        idx = int(np.argmax(self.steps[-1]["logp"]))
        for k in range(len(self.steps) - 1, 0, -1):
            idx = int(self.steps[k]["back"][idx])
            if idx < 0:
                return None
        return self._match(self.steps[0], idx, 1.0)
