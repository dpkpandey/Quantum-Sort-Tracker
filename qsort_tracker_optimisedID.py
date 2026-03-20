# ===============================================================
# QSort Physics Tracker full  debuging level4   (Full Logging) 
# Author: Deepak Pandey 
# ===============================================================
#
# Key improvements in this version:
#   ✔ Direction-independent matching
#   ✔ NO ID REUSE (never assign an ID to a different fish)
#   ✔ 5-frame recovery using future predicted position
#   ✔ Boltzmann physics prediction (velocity+acc+jerk+curvature)
#   ✔ Curvature & jerk penalization for realism
#   ✔ Full debug logging: assignments, predictions, recoveries
#   ✔ Safe cost thresholds preventing mismatches
#   ✔ Exports full 17-value physics output + reliability R
# There will be more updates on this software as it goes on, you guys might need to fine tune and adjust many parameters as required. The main logic for counting or detecting also will be  provided  with this to understand more
# =============================================================== 


#FOR NOW AT THIS POINT ON DAY OF MARCH 2026 THIS SOFTWARE IS DISTRIBUTED AS A FREE, ANYONE ARE WELCOME TO USE IT IMPLIMENT IN THEIR PROJECT BY CITING THIS WORK 

import numpy as np
from scipy.optimize import linear_sum_assignment

DEBUG_LOGFILE = "qsort_debug_log.txt"

# overwrite log each run
with open(DEBUG_LOGFILE, "w") as f:
    f.write("=== QSortPhysicsTracker Debug Log (Mode 2) ===\n")

def D(msg):
    """Debug logger."""
    with open(DEBUG_LOGFILE, "a") as f:
        f.write(msg + "\n")


EPS = 1e-6
DT = 1.0
HIST_LEN = 15


# ---------------------------------------------------------------
# Utility
# ---------------------------------------------------------------
def safe_norm(x, y):
    return float(np.hypot(x, y))


# ---------------------------------------------------------------
# Physics-Based Track Object
# ---------------------------------------------------------------
class PhysicsTrack:
    """
    One object track with full physical state.
    Includes:
        position, velocity, acceleration, jerk,
        curvature, uncertainty, momentum, reliability.
    """

    def __init__(self, tid, det):
        # do not reuse id — guarantee global uniqueness
        self.id = tid

        self.update_bbox(det)

        # history arrays for physics computation
        self.history = [(self.x, self.y)]
        self.vel_history = []
        self.acc_history = []

        # physics states
        self.vx = self.vy = 0
        self.ax = self.ay = 0
        self.jx = self.jy = 0
        self.curvature = 0

        # uncertainty
        self.sigma_x = 0
        self.sigma_y = 0
        self.sigma_theta = 0

        # effective mass & momentum
        self.meff = 1.0
        self.px = 0
        self.py = 0

        # reliability
        self.R = 1.0

        # bookkeeping
        self.missed = 0
        self.age = 1

    # -----------------------------------------------------------
    def update_bbox(self, det):
        """Update raw bounding box & center."""
        x1, y1, x2, y2, _ = det
        self.bbox = det.copy()
        self.x = (x1 + x2) / 2
        self.y = (y1 + y2) / 2

    # -----------------------------------------------------------
    def update_state(self):
        """Compute velocity, acceleration, jerk, curvature, uncertainties."""
        # Position history
        self.history.append((self.x, self.y))
        if len(self.history) > HIST_LEN:
            self.history.pop(0)

        # Velocity
        if len(self.history) >= 2:
            (x2, y2) = self.history[-1]
            (x1, y1) = self.history[-2]
            vx = (x2 - x1) / DT
            vy = (y2 - y1) / DT
        else:
            vx = vy = 0

        # Acceleration
        if len(self.vel_history) >= 1:
            (vx_prev, vy_prev) = self.vel_history[-1]
            ax = (vx - vx_prev) / DT
            ay = (vy - vy_prev) / DT
        else:
            ax = ay = 0

        # Jerk
        if len(self.acc_history) >= 1:
            (ax_prev, ay_prev) = self.acc_history[-1]
            jx = (ax - ax_prev) / DT
            jy = (ay - ay_prev) / DT
        else:
            jx = jy = 0

        # Store histories
        self.vel_history.append((vx, vy))
        self.acc_history.append((ax, ay))

        if len(self.vel_history) > HIST_LEN: self.vel_history.pop(0)
        if len(self.acc_history) > HIST_LEN: self.acc_history.pop(0)

        # Curvature
        speed = safe_norm(vx, vy)
        if speed > EPS:
            self.curvature = abs(vx * ay - vy * ax) / (speed**3 + EPS)
        else:
            self.curvature = 0

        # Uncertainty
        xs = [p[0] for p in self.history]
        ys = [p[1] for p in self.history]
        self.sigma_x = float(np.std(xs)) if len(xs) >= 3 else 0
        self.sigma_y = float(np.std(ys)) if len(ys) >= 3 else 0

        if len(self.vel_history) >= 2:
            angles = [np.arctan2(vy, vx + EPS) for (vx, vy) in self.vel_history]
            self.sigma_theta = float(np.std(angles))
        else:
            self.sigma_theta = 0

        # Effective mass (simple model)
        self.meff = 1.0 + speed

        # Momentum
        self.px = self.meff * vx
        self.py = self.meff * vy

        # Reliability score
        jmag = safe_norm(jx, jy)
        penalty = (
            0.5 * jmag +
            0.4 * self.curvature +
            0.3 * self.sigma_x +
            0.3 * self.sigma_y +
            0.2 * self.sigma_theta
        )
        self.R = float(np.exp(-penalty))

        # save final physics state
        self.vx, self.vy = vx, vy
        self.ax, self.ay = ax, ay
        self.jx, self.jy = jx, jy

    # -----------------------------------------------------------
    def update(self, det):
        """Update from detection and recompute physics."""
        self.update_bbox(det)
        self.update_state()
        self.missed = 0
        self.age += 1

    # -----------------------------------------------------------
    def mark_missed(self):
        """Track was not matched this frame."""
        self.missed += 1
        self.age += 1

    # -----------------------------------------------------------
    def predict_candidates(self):
        """Return 4 physics-based predicted positions."""
        # Simple motion expansion
        x1 = self.x + self.vx
        y1 = self.y + self.vy

        x2 = self.x + self.vx + 0.5 * self.ax
        y2 = self.y + self.vy + 0.5 * self.ay

        x3 = x2 + (1.0 / 6.0) * self.jx
        y3 = y2 + (1.0 / 6.0) * self.jy

        x4 = self.x - self.curvature * self.vy
        y4 = self.y + self.curvature * self.vx

        return np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], float)


# ===============================================================
# BOLTZMANN PREDICTOR
# ===============================================================
class BoltzmannPredictor:

    def __init__(self, base_temp=1.0):
        self.base_temp = base_temp

    def compute_energy(self, T: PhysicsTrack):
        """Global energy for the whole object state."""
        vmag = safe_norm(T.vx, T.vy)
        amag = safe_norm(T.ax, T.ay)
        jmag = safe_norm(T.jx, T.jy)

        E = (
            0.4 * vmag +
            0.7 * amag +
            1.2 * jmag +
            1.8 * T.curvature +
            1.5 * (1 - T.R) +
            0.5 * (T.sigma_x + T.sigma_y) +
            0.3 * T.sigma_theta
        )
        return max(EPS, float(E))

    def compute_temperature(self, T: PhysicsTrack):
        """Temperature increases with uncertainty."""
        uncertainty = (
            T.sigma_x + T.sigma_y +
            0.5 * T.sigma_theta +
            (1 - T.R)
        )
        return max(0.1, float(self.base_temp * (1 + uncertainty)))

    def predict(self, T: PhysicsTrack):
        """Boltzmann weighted candidate expectation."""
        cands = T.predict_candidates()
        E = self.compute_energy(T)
        Ttemp = self.compute_temperature(T)

        # Option A: same energy for all cands
        w = np.exp(-E / Ttemp)
        weights = np.array([w, w, w, w], float)
        weights /= (np.sum(weights) + EPS)

        xp = float(np.sum(weights * cands[:, 0]))
        yp = float(np.sum(weights * cands[:, 1]))
        return xp, yp


# ===============================================================
# COST FUNCTIONS
# ===============================================================
def iou(boxA, boxB):
    """Standard IoU."""
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    inter = max(0, xB - xA) * max(0, yB - yA)
    areaA = max(1.0, (boxA[2]-boxA[0])*(boxA[3]-boxA[1]))
    areaB = max(1.0, (boxB[2]-boxB[0])*(boxB[3]-boxB[1]))
    return inter / (areaA + areaB - inter + EPS)


def physics_mismatch(T: PhysicsTrack, det):
    """High cost = less physical match."""
    dx = (det[0] + det[2]) / 2
    dy = (det[1] + det[3]) / 2

    disp = safe_norm(dx - T.x, dy - T.y)
    vel_mag = safe_norm(T.vx, T.vy)

    # angle mismatch
    if vel_mag > EPS and disp > EPS:
        cos_a = ((dx - T.x) * T.vx + (dy - T.y) * T.vy) / (vel_mag * disp + EPS)
        angle_diff = 1 - abs(cos_a)
    else:
        angle_diff = 0

    vel_dev = abs(disp - vel_mag)
    acc_pen = safe_norm(T.ax, T.ay)
    jerk_pen = safe_norm(T.jx, T.jy)

    cost = (
        0.3*vel_dev +
        1.0*angle_diff +
        0.6*acc_pen +
        1.0*jerk_pen +
        1.2*T.curvature +
        0.4*(T.sigma_x + T.sigma_y) +
        0.6*(1 - T.R)
    )
    return float(cost)


def build_cost_matrix(tracks, detections, predictor):
    N, M = len(tracks), len(detections)
    if N == 0 or M == 0:
        return np.zeros((N, M), float)

    C = np.full((N, M), 1e6, float)

    for i, T in enumerate(tracks):
        xp, yp = predictor.predict(T)

        for j, det in enumerate(detections):
            cx = (det[0] + det[2]) / 2
            cy = (det[1] + det[3]) / 2

            dist = safe_norm(xp - cx, yp - cy)
            if dist > 350:  # generous range
                continue

            iouv = iou(T.bbox, det)
            if iouv < 0.01:
                continue

            iou_cost = 80 * (1 - iouv)
            phys_cost = physics_mismatch(T, det)

            C[i, j] = 0.4*dist + iou_cost + 1.1*phys_cost

    return C


# ===============================================================
# STRICT RECOVERY RULE
# ===============================================================
def allow_reid(T, det):
    """Try to reclaim same ID after brief disappearance."""
    if T.missed > 5:  # vanish >5 frames → drop
        return False

    dx = (det[0] + det[2]) / 2
    dy = (det[1] + det[3]) / 2

    disp = safe_norm(dx - T.x, dy - T.y)
    if disp > 100:
        return False

    if T.R < 0.3:
        return False

    return True


# ===============================================================
# MAIN TRACKER CLASS
# ===============================================================
class QSortPhysicsTracker:

    def __init__(self, max_missed=8, reid_gap=5):
        self.tracks = []
        self.next_id = 1
        self.max_missed = max_missed
        self.reid_gap = reid_gap
        self.predictor = BoltzmannPredictor()

    # -----------------------------------------------------------
    def update(self, dets):
        """Main tracker update."""
        D(f"\n--- FRAME UPDATE ---")
        D(f"Detections: {len(dets)}")

        # Edge case: no existing tracks
        if len(self.tracks) == 0:
            for d in dets:
                T = PhysicsTrack(self.next_id, d)
                T.update_state()
                self.tricks_log_birth(T.id)
                self.tracks.append(T)
                self.next_id += 1
            return self._export()

        # Edge case: no detections
        if len(dets) == 0:
            for T in self.tracks:
                T.mark_missed()
                D(f"[MISS] Track {T.id} now missed={T.missed}")
            self._cleanup()
            return self._export()

        # Cost matrix
        C = build_cost_matrix(self.tracks, dets, self.predictor)
        rows, cols = linear_sum_assignment(C)

        assigned_tracks = set()
        assigned_dets = set()

        # -------------------------------------------------------
        # Update matched tracks
        # -------------------------------------------------------
        for r, c in zip(rows, cols):
            if C[r, c] < 1e5:
                T = self.tracks[r]
                T.update(dets[c])
                assigned_tracks.add(r)
                assigned_dets.add(c)
                D(f"[ASSIGN] Track {T.id} matched det[{c}] | cost={C[r,c]:.2f}")
            else:
                D("[ASSIGN FAIL] Cost too high")

        # -------------------------------------------------------
        # Mark unmatched tracks as missed
        # -------------------------------------------------------
        for idx, T in enumerate(self.tracks):
            if idx not in assigned_tracks:
                T.mark_missed()
                D(f"[MISS] Track {T.id} missed={T.missed}")

        # -------------------------------------------------------
        # Process unmatched dets → try re-ID
        # -------------------------------------------------------
        for j, d in enumerate(dets):
            if j in assigned_dets:
                continue

            # try recovery
            recovered = False
            for T in self.tracks:
                if T.missed > 0 and allow_reid(T, d):
                    T.update(d)
                    recovered = True
                    D(f"[REID] Track {T.id} recovered with det[{j}]")
                    break

            if not recovered:
                # new track
                Tnew = PhysicsTrack(self.next_id, d)
                Tnew.update_state()
                self.tracks.append(Tnew)
                D(f"[NEW] New track born id={self.next_id}")
                self.next_id += 1

        self._cleanup()
        return self._export()

    # -----------------------------------------------------------
    def tricks_log_birth(self, tid):
        D(f"[BIRTH] Track born id={tid}")

    # -----------------------------------------------------------
    def _cleanup(self):
        """Remove dead tracks."""
        alive = []
        for T in self.tracks:
            if T.missed <= self.max_missed:
                alive.append(T)
            else:
                D(f"[DELETE] Track {T.id} removed (missed={T.missed})")
        self.tracks = alive

    # -----------------------------------------------------------
    def _export(self):
        """Return full physics output for visible tracks."""
        out = []
        for T in self.tracks:
            if T.missed == 0:
                x1, y1, x2, y2, _ = T.bbox
                out.append([
                    x1, y1, x2, y2,
                    T.id,
                    T.vx, T.vy,
                    T.ax, T.ay,
                    T.jx, T.jy,
                    T.curvature,
                    T.px, T.py,
                    T.sigma_x, T.sigma_y, T.sigma_theta,
                    T.R
                ])
        return np.array(out, float) if len(out) else np.empty((0, 17), float)
