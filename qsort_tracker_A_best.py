import numpy as np
from scipy.optimize import linear_sum_assignment

# ===============================================================
# Helper Functions
# ===============================================================

def iou(b1, b2):
    x1 = max(b1[0], b2[0])
    y1 = max(b1[1], b2[1])
    x2 = min(b1[2], b2[2])
    y2 = min(b1[3], b2[3])

    inter = max(0, x2 - x1) * max(0, y2 - y1)
    a1 = max(0, b1[2] - b1[0]) * max(0, b1[3] - b1[1])
    a2 = max(0, b2[2] - b2[0]) * max(0, b2[3] - b2[1])
    union = a1 + a2 - inter + 1e-6
    return inter / union


def center(det):
    x1, y1, x2, y2, _ = det
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def motion_stats(history):
    if len(history) < 3:
        return dict(v=0.0, a=0.0, j=0.0, curv=0.0)

    pts = np.array(history, dtype=float)
    dv = pts[1:] - pts[:-1]          # velocities
    v_mag = np.linalg.norm(dv, axis=1)

    da = dv[1:] - dv[:-1]            # accelerations
    a_mag = np.linalg.norm(da, axis=1) if len(da) else [0.0]

    if len(da) > 1:
        j = da[1:] - da[:-1]         # jerks
        j_mag = np.linalg.norm(j, axis=1)
        jerk_val = float(j_mag[-1])
    else:
        jerk_val = 0.0

    # curvature κ ≈ |v × a| / |v|³
    if len(dv) > 1 and len(da) > 0:
        vx, vy = dv[-1]
        ax, ay = da[-1]
        num = abs(vx * ay - vy * ax)
        den = (v_mag[-1] ** 3) + 1e-6
        curv = float(num / den)
    else:
        curv = 0.0

    return dict(
        v=float(v_mag[-1]),
        a=float(a_mag[-1] if len(a_mag) else 0.0),
        j=float(jerk_val),
        curv=curv
    )


# ===============================================================
# Track Object
# ===============================================================

class QTrackA:
    def __init__(self, tid, det, frame_idx):
        self.id = tid
        self.bbox = np.array(det, dtype=float)
        self.cx, self.cy = center(det)
        self.history = [(self.cx, self.cy)]
        self.stats = motion_stats(self.history)
        self.age = 1
        self.missed = 0
        self.last_update_frame = frame_idx

    def predict(self):
        if len(self.history) < 2:
            return self.cx, self.cy
        vx = self.history[-1][0] - self.history[-2][0]
        vy = self.history[-1][1] - self.history[-2][1]
        return self.cx + vx, self.cy + vy

    def update(self, det, frame_idx):
        self.bbox = np.array(det, dtype=float)
        self.cx, self.cy = center(det)
        self.history.append((self.cx, self.cy))
        if len(self.history) > 15:
            self.history.pop(0)
        self.stats = motion_stats(self.history)
        self.missed = 0
        self.age += 1
        self.last_update_frame = frame_idx

    def mark_missed(self):
        self.missed += 1
        self.age += 1


# ===============================================================
# QSortTracker A
# ===============================================================

class QSortTracker:
    """
    Quantum-inspired SORT-like tracker:
    - Uses IoU + distance for matching (SORT-style core)
    - Adds small penalties from velocity / jerk
    - Keeps a short memory of recently-collapsed tracks
    - During a short "freeze window", prevents spawning new IDs
      exactly where a track just disappeared → avoids flip re-ID.
    """

    def __init__(
        self,
        max_missed=8,
        max_dist=120.0,
        min_iou=0.03,
        freeze_new_id_frames=3
    ):
        self.tracks = []
        self.next_id = 1

        self.max_missed = max_missed
        self.max_dist = max_dist
        self.min_iou = min_iou
        self.freeze_new_id_frames = freeze_new_id_frames

        self.frame_idx = 0
        # recent collapses: (tid, cx, cy, frame_idx)
        self.collapsed = []
        self.no_new_id_until = -1

    # ---------- cost matrix ----------
    def _cost_matrix(self, tracks, dets):
        C = np.zeros((len(tracks), len(dets)), dtype=float)

        for i, T in enumerate(tracks):
            px, py = T.predict()
            Tb = T.bbox

            for j, D in enumerate(dets):
                cx, cy = center(D)
                d = np.hypot(cx - px, cy - py)

                if d > self.max_dist:
                    C[i, j] = 1e6
                    continue

                iou_val = iou(Tb, D)
                if iou_val < self.min_iou:
                    C[i, j] = 1e6
                    continue

                # SORT-like base
                iou_cost = (1.0 - iou_val) * 100.0
                dist_cost = d * 0.4

                # Small penalties from motion
                v_pen = abs(T.stats["v"]) * 0.08
                j_pen = abs(T.stats["j"]) * 0.05

                C[i, j] = iou_cost + dist_cost + v_pen + j_pen

        return C

    def _near_collapse(self, cx, cy):
        """
        Check if (cx,cy) lies near a recently collapsed track
        within the freeze window. If yes, return that track id.
        """
        for tid, ox, oy, fidx in self.collapsed:
            if self.frame_idx - fidx <= self.freeze_new_id_frames:
                if np.hypot(cx - ox, cy - oy) < self.max_dist:
                    return tid
        return None

    def update(self, detections):
        self.frame_idx += 1
        dets = np.array(detections, dtype=float) if len(detections) else np.empty((0, 5))

        # === CASE 1: no existing tracks ===
        if len(self.tracks) == 0:
            for d in dets:
                self.tracks.append(QTrackA(self.next_id, d, self.frame_idx))
                self.next_id += 1
            return self._export()

        # === CASE 2: no detections ===
        if len(dets) == 0:
            for T in self.tracks:
                T.mark_missed()
                if T.missed == 1:
                    # record first time collapse
                    self.collapsed.append((T.id, T.cx, T.cy, self.frame_idx))
                    self.no_new_id_until = self.frame_idx + self.freeze_new_id_frames
            self._cleanup()
            return np.empty((0, 8))

        # === STEP 1: Assign with Hungarian ===
        C = self._cost_matrix(self.tracks, dets)
        rows, cols = linear_sum_assignment(C)

        assigned_T = set()
        assigned_D = set()

        for r, c in zip(rows, cols):
            if C[r, c] >= 1e6:
                continue
            self.tracks[r].update(dets[c], self.frame_idx)
            assigned_T.add(r)
            assigned_D.add(c)

        # === STEP 2: Mark missed tracks & remember collapses ===
        for idx, T in enumerate(self.tracks):
            if idx not in assigned_T:
                T.mark_missed()
                if T.missed == 1:
                    self.collapsed.append((T.id, T.cx, T.cy, self.frame_idx))
                    self.no_new_id_until = self.frame_idx + self.freeze_new_id_frames

        # === STEP 3: Handle unassigned detections ===
        for j, D in enumerate(dets):
            if j in assigned_D:
                continue

            cx, cy = center(D)

            # Try to recover a collapsed track
            collapsed_tid = self._near_collapse(cx, cy)
            if collapsed_tid is not None:
                for T in self.tracks:
                    if T.id == collapsed_tid:
                        T.update(D, self.frame_idx)
                        T.missed = 0
                        break
                continue

            # Inside freeze window: don't spawn brand-new IDs in the same area
            if self.frame_idx <= self.no_new_id_until:
                continue

            # Otherwise: allow new ID (e.g., stacked fish separates)
            self.tracks.append(QTrackA(self.next_id, D, self.frame_idx))
            self.next_id += 1

        self._cleanup()
        return self._export()

    def _cleanup(self):
        # remove dead tracks
        self.tracks = [T for T in self.tracks if T.missed <= self.max_missed]
        # keep collapse memory short
        self.collapsed = [
            c for c in self.collapsed
            if self.frame_idx - c[3] <= self.freeze_new_id_frames
        ]

    def _export(self):
        """
        Returns ndarray of live tracks:
        [x1, y1, x2, y2, id, v, curv, j]
        """
        out = []
        for T in self.tracks:
            if T.missed != 0:
                continue
            x1, y1, x2, y2, conf = T.bbox
            out.append([
                x1, y1, x2, y2,
                T.id,
                T.stats["v"],
                T.stats["curv"],
                T.stats["j"],
            ])
        return np.array(out, dtype=float) if len(out) else np.empty((0, 8), dtype=float)
