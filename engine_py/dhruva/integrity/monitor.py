"""GNSS trust guard: decides whether each fix is applied, and reports the navigation mode.

States: LOCK (fixes drive the filter), DR (no usable GNSS), REACQUIRE (fixes are back but not yet trusted),
SPOOF_REJECTED (fixes contradict the filter and are withheld). Master spec section 7.4.

Fixes are never applied while not in LOCK. Re-entry needs several consecutive fixes that agree with each other, with
the filter's speed and course, and with the filter's covariance gate. After a plain outage (not a rejected spoof),
a long run of mutually consistent fixes is accepted even outside the gate, so an over-tight covariance cannot
deadlock re-entry. A rejected spoof never gets that shortcut.
"""
from dataclasses import dataclass

import numpy as np

APPLY, WITHHOLD = "apply", "withhold"
MODE_OF_STATE = {"LOCK": "GNSS", "DR": "DR", "REACQUIRE": "DR", "SPOOF_REJECTED": "SPOOF_REJECTED"}


@dataclass
class FixInfo:
    t: float
    e: float           # ENU position of the fix
    n: float
    acc: float         # reported horizontal accuracy [m]
    sats: float
    speed: float
    bearing_psi: float  # course as psi (rad, CCW from East); NaN if unavailable


@dataclass
class Prediction:
    nis: float          # position innovation NIS of this fix against the current filter
    cov95_m: float
    speed: float        # filter speed
    psi: float          # filter yaw
    disp: object        # callable (t_a, t_b) -> filter displacement (dE, dN) between two times, or None


class IntegrityMonitor:
    def __init__(self, cfg):
        self.c = cfg["integrity"]
        self.state = "LOCK"
        self.origin = None                 # "DR" or "SPOOF": what we are re-acquiring from
        self.events = []                   # (t, kind, detail)
        self.last_seen_t = None            # last fix RECEIVED (accepted or not): "no fix" means no fix at all
        self._bad = 0
        self._bad_min = 0.0
        self._grace_until = -np.inf        # after a (re-)lock the filter may still be converging
        self._state_t = 0.0                # when the current state was entered
        self._cusum = 0.0
        self._ok = 0                       # consecutive acceptable candidates while re-acquiring
        self._prev = None                  # previous candidate fix
        self._forced = 0                   # consecutive mutually consistent candidates (ignores the gate)
        self.reentered = False             # True on the fix that switched to LOCK

    # ---- outputs -----------------------------------------------------------
    @property
    def mode(self):
        return MODE_OF_STATE[self.state]

    @property
    def trust(self):
        if self.state == "LOCK":
            return float(np.clip(1.0 - self._cusum / self.c["cusum_h"], 0.0, 1.0))
        if self.state == "REACQUIRE" or (self.state == "SPOOF_REJECTED" and self._ok):
            return float(min(self._ok / self.c["reacquire_consecutive_fixes"], 1.0)) * 0.5
        return 0.0

    def _event(self, t, kind, detail=""):
        self.events.append((float(t), kind, detail))

    def _to(self, state, t, why, origin=None):
        if state != self.state:
            self._event(t, state, why)
            self._state_t = t
        self.state = state
        if origin is not None:
            self.origin = origin
        self._ok, self._forced, self._prev = 0, 0, None

    # ---- inputs --------------------------------------------------------------
    def on_tick(self, t):
        """Called regularly; drops to DR when fixes stop arriving."""
        if self.state == "LOCK" and self.last_seen_t is not None \
                and t - self.last_seen_t > self.c["no_fix_timeout_s"] + 0.05:
            self._to("DR", t, "no fix", origin="DR")

    def on_seen(self, t):
        self.last_seen_t = t

    def on_fix(self, fix, pred):
        """Decide APPLY or WITHHOLD for a fix. Sets self.reentered when a re-acquisition completes."""
        self.reentered = False
        c, t = self.c, fix.t
        self.last_seen_t = t
        if fix.sats < c["min_sats"] or fix.acc > c["max_acc_h_m"]:
            if self.state == "LOCK":
                self._to("DR", t, "poor fix quality", origin="DR")
            return WITHHOLD
        if self.state == "LOCK":
            return self._lock(fix, pred)
        return self._reacquire(fix, pred)

    # ---- LOCK ----------------------------------------------------------------
    def _lock(self, fix, pred):
        c, t = self.c, fix.t
        if pred.nis > c["chi2_99_2dof"]:
            self._bad += 1
            self._bad_min = pred.nis if self._bad == 1 else min(self._bad_min, pred.nis)
            if self._bad >= c["spoof_consecutive_fixes"]:
                clear_spoof = (pred.cov95_m < c["spoof_cov95_max_m"] and self._bad_min >= c["spoof_hard_nis"]
                               and t >= self._grace_until)
                if clear_spoof:
                    self._to("SPOOF_REJECTED", t, f"NIS >= {self._bad_min:.0f} on {self._bad} consecutive fixes",
                             origin="SPOOF")
                else:                      # borderline misfit or an uncertain filter: verify before trusting
                    # A confident filter with a borderline misfit is a suspect, not an outage: no deadlock shortcut.
                    origin = "SUSPECT" if pred.cov95_m < c["spoof_cov95_max_m"] else "DR"
                    self._to("REACQUIRE", t, f"NIS {pred.nis:.0f} on {self._bad} fixes, not clearly a spoof",
                             origin=origin)
                self._bad = 0
            return WITHHOLD
        self._bad = 0
        self._cusum = max(0.0, self._cusum + pred.nis - 2.0 - c["cusum_k"])
        if self._cusum > c["cusum_h"]:
            self._to("SPOOF_REJECTED", t, f"CUSUM {self._cusum:.0f}", origin="SPOOF")
            self._cusum = 0.0
            return WITHHOLD
        return APPLY

    # ---- DR / REACQUIRE / SPOOF_REJECTED ----------------------------------------
    def _consistent(self, fix, pred):
        c = self.c
        if fix.speed > 1.0 and not np.isnan(fix.bearing_psi):
            dpsi = abs((fix.bearing_psi - pred.psi + np.pi) % (2 * np.pi) - np.pi)
            if abs(fix.speed - pred.speed) > c["reacquire_speed_tol_mps"] or \
                    np.degrees(dpsi) > c["reacquire_course_tol_deg"]:
                return False
        elif abs(fix.speed - pred.speed) > c["reacquire_speed_tol_mps"]:
            return False
        p = self._prev
        if p is not None and pred.disp is not None and fix.t - p.t < 5.0:
            dE, dN = pred.disp(p.t, fix.t)
            tol = c["reacquire_pos_sigma_mult"] * np.sqrt(2.0) * max(fix.acc, 3.0) + c["reacquire_pos_slack_m"]
            if np.hypot((fix.e - p.e) - dE, (fix.n - p.n) - dN) > tol:
                return False
        return True

    def _gnss_consistent(self, fix):
        """Fixes agree with each other and with their own reported velocity (no reference to the filter)."""
        p = self._prev
        if p is None or fix.t - p.t >= 5.0 or fix.t <= p.t:
            return True
        c = self.c
        moving = fix.speed > 1.0 and p.speed > 1.0 and not np.isnan(fix.bearing_psi) and not np.isnan(p.bearing_psi)
        if moving:
            v = 0.5 * (p.speed * np.array([np.cos(p.bearing_psi), np.sin(p.bearing_psi)])
                       + fix.speed * np.array([np.cos(fix.bearing_psi), np.sin(fix.bearing_psi)]))
            expected = v * (fix.t - p.t)
        else:
            expected = np.zeros(2)
        tol = c["reacquire_pos_sigma_mult"] * np.sqrt(2.0) * max(fix.acc, 3.0) + c["reacquire_pos_slack_m"]
        return bool(np.hypot(fix.e - p.e - expected[0], fix.n - p.n - expected[1]) <= tol)

    def _reacquire(self, fix, pred):
        c, t = self.c, fix.t
        if self.state == "DR":
            self._to("REACQUIRE", t, "fixes present again")
        if self.state == "REACQUIRE" and self.origin == "SUSPECT" and t - self._state_t > c["suspect_escalate_s"]:
            self._to("SPOOF_REJECTED", t, "borderline misfit persisted", origin="SPOOF")
        consistent = self._consistent(fix, pred)
        in_gate = pred.nis <= c["chi2_99_2dof"]
        self._ok = self._ok + 1 if (consistent and in_gate) else 0
        self._forced = self._forced + 1 if self._gnss_consistent(fix) else 0
        self._prev = fix
        forced = self.origin == "DR" and self._forced >= c["reacquire_force_accept_fixes"]
        if self._ok >= c["reacquire_consecutive_fixes"] or forced:
            self._event(t, "LOCK", "re-acquired" + (" (consistency only)" if forced and not in_gate else ""))
            self.state, self._ok, self._forced, self._prev = "LOCK", 0, 0, None
            self._bad, self._cusum, self.reentered = 0, 0.0, True
            self._grace_until = t + c["post_lock_grace_s"]
            return APPLY
        return WITHHOLD
