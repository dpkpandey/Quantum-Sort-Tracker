# ================================================================
# QSort Physics Tracker --- Full 6-Layer Implementation updated recently 
# Author: Deepak Pandey
# Reviewed & fixed by: Auther
#
# LAYERS:
#   L1 — QPCMSV extended classical state vector (16D)
#   L2 — Polynomial regression nonlinear prediction
#   L3 — Wavepacket uncertainty spreading + collapse
#   L4 — Boltzmann spatial probability field
#   L5 — Bloch-sphere regime encoding (3 qubits)
#   L6 — Multi-qubit motion tensor (8D regime fusion)
# Most important part is in this code, most of the tracking system is still naive and untested yet in different datasets, however I just published because of time rectriction, if I may not continue 
#It would be awesome if someone wants to enhance this code and make it better, but you might need littlebit knowledge of basic physics and quanutm mechanics. Where most of the approaches are defined in such a way
#But it does not mean that it needs quantum hardware, it is just concept I borrow from there but I have also now idea whether it works or not. But while testing on my project it was ripper and worked so good. 


import numpy as np
import logging
import threading
from scipy.optimize import linear_sum_assignment
from collections import deque

# ----------------------------------------------------------------
# ABLATION FLAGS
# ----------------------------------------------------------------
USE_L2_POLYNOMIAL   = True
USE_L3_WAVEPACKET   = True
USE_L4_BOLTZMANN    = True
USE_L5_BLOCH        = True
USE_L6_TENSOR       = True

# ----------------------------------------------------------------
# DEBUG LOGGING — [H5] default OFF, uses Python logging not raw file I/O
# ----------------------------------------------------------------
DEBUG_LOG = False
_log = logging.getLogger("qsort")

def _D(msg):
    if DEBUG_LOG:
        _log.debug(msg)

# ----------------------------------------------------------------
# FINE-TUNING PARAMETERS
# ----------------------------------------------------------------
DT              = 1.0        # time step (frames)
HIST_LEN        = 15         # max history frames
EPS             = 1e-6

# Detection gate — [M6] filter weak detections before they enter cost matrix
MIN_CONF        = 0.25       # minimum YOLO confidence score

# Layer 2 — Polynomial regression
POLY_DEGREE     = 3
POLY_WIN_HIGH_R = 12
POLY_WIN_MED_R  = 7
POLY_WIN_LOW_R  = 4
POLY_LAMBDA     = 0.005

# Layer 3 — Wavepacket
SIGMA_INIT      = 4.0
SIGMA_MIN       = 0.5
DIFFUSION_BASE  = 1.5
DIFFUSION_CURV  = 1.0

# Layer 4 — Boltzmann
BOLTZ_ALPHA     = 0.20       # acceleration mismatch weight in energy
BOLTZ_BETA      = 0.25       # velocity mismatch weight
BOLTZ_GAMMA     = 0.10       # curvature mismatch weight
TEMP_INIT       = 0.5
TEMP_MIN        = 0.3
TEMP_HEAT_RATE  = 0.04

# [H3] Distance gate — expressed as fraction of frame diagonal
# Computed in px at tracker init from (frame_w, frame_h)
# Default covers ~25% of a 1080p diagonal
BOLTZ_DIST_FRAC = 0.16       # fraction of frame diagonal

# Layer 5 — Bloch spheres
BLOCH_TAU       = 4.0
BLOCH_K         = 0.20
# [H2] Speed thresholds scaled for real fish video (px/frame at typical res)
SPEED_S1        = 2.0        # stopped → gliding  (was 0.1 — 20× too small)
SPEED_S2        = 15.0       # gliding → burst    (was 0.5 — 30× too small)

# Layer 6 — Tensor coupling
TENSOR_LAMBDA1  = 0.3
TENSOR_LAMBDA2  = 0.2
TENSOR_LAMBDA3  = 0.2

# [L1] Tensor alpha coupling — now exposed as tunable parameters
# Index [i,j,k]: i=direction active, j=turn active, k=burst active
TENSOR_ALPHA = np.ones((2, 2, 2), float)
TENSOR_ALPHA[1, 1, 0] = 1.2   # direction + turn (straight curve)
TENSOR_ALPHA[0, 1, 1] = 1.2   # turn + burst (sudden direction change)
TENSOR_ALPHA[1, 0, 1] = 1.0   # direction + burst (straight sprint)
TENSOR_ALPHA[1, 1, 1] = 1.5   # all three — strongest coupling

# [C3] Association cost weights — all terms normalised to [0,1] before weighting
COST_DIST_W     = 0.35
COST_BOLTZ_W    = 0.25
COST_DIR_W      = 0.20       # now visible because terms are normalised
COST_TENSOR_W   = 0.10       # now visible
COST_IOU_W      = 0.10       # inverse IoU, normalised
IOU_MIN         = 0.01       # minimum IoU to pass gate

# Reliability — [C2] single unified system
R_INIT          = 0.80
R_MAX           = 1.00
R_BOOST         = 0.04       # per matched frame
R_DECAY         = 0.03       # per missed frame (applied once, in predict())
R_FLOOR_ACTIVE  = 0.12       # delete if R below this AND missed > 0
MIN_AGE_DELETE  = 4          # never delete younger than this

# Track birth/death
MAX_MISSED      = 50
BIRTH_CONFIRM   = 2          # matches needed before first export

# [M4] Thread-safe ID counter
_id_lock = threading.Lock()
_global_id_counter = 1

def _next_id():
    global _global_id_counter
    with _id_lock:
        tid = _global_id_counter
        _global_id_counter += 1
    return tid

def reset_id_counter(start=1):
    """Call between independent tracking sessions."""
    global _global_id_counter
    with _id_lock:
        _global_id_counter = start


# ================================================================
# UTILITIES
# ================================================================
def safe_norm(x, y):
    return float(np.hypot(x, y))

def iou(boxA, boxB):
    xA = max(boxA[0], boxB[0]); yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2]); yB = min(boxA[3], boxB[3])
    inter = max(0.0, xB - xA) * max(0.0, yB - yA)
    aA = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    aB = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    if aA <= 0 or aB <= 0:
        return 0.0
    return inter / (aA + aB - inter + EPS)


# ================================================================
# LAYER 2 — Polynomial Regression (real history only)
# ================================================================
class PolynomialPredictor:

    def fit(self, times, positions, degree, lam=POLY_LAMBDA):
        if len(times) < 2:
            return None
        deg = min(degree, len(times) - 1)
        T = np.vstack([times**i for i in range(deg + 1)]).T
        TtT = T.T @ T
        try:
            coeffs = np.linalg.solve(TtT + lam * np.eye(deg + 1), T.T @ positions)
        except np.linalg.LinAlgError:
            return None
        return coeffs

    def predict_at(self, coeffs, t):
        if coeffs is None:
            return None, None, None, None
        n = len(coeffs)
        pos  = sum(coeffs[i] * t**i                     for i in range(n))
        vel  = sum(coeffs[i] * i * t**(i-1)             for i in range(1, n))
        acc  = sum(coeffs[i] * i*(i-1) * t**(i-2)       for i in range(2, n))
        jerk = sum(coeffs[i] * i*(i-1)*(i-2) * t**(i-3) for i in range(3, n))
        return float(pos), float(vel), float(acc), float(jerk)

    def window_size(self, R):
        if R > 0.7: return POLY_WIN_HIGH_R
        if R > 0.4: return POLY_WIN_MED_R
        return POLY_WIN_LOW_R


# ================================================================
# LAYER 3 — Wavepacket
# ================================================================
class WavepacketEngine:

    def diffusion(self, R, kappa):
        if not USE_L3_WAVEPACKET:
            return 0.0
        return DIFFUSION_BASE * (2.0 - R) + DIFFUSION_CURV * kappa

    def spread(self, sigma, D):
        return float(np.sqrt(sigma**2 + D * DT))

    def collapse_to(self, x, y, px, py):
        phi = float(px * x + py * y)
        return x, y, SIGMA_MIN, phi


# ================================================================
# LAYER 4 — Boltzmann Field
# ================================================================
class BoltzmannEngine:

    def energy(self, r, mu, sigma_eff, dv, da, dk):
        disp = ((r[0]-mu[0])**2 + (r[1]-mu[1])**2) / (2*sigma_eff**2 + EPS)
        return float(disp + BOLTZ_BETA*dv + BOLTZ_ALPHA*da + BOLTZ_GAMMA*dk)

    def best_position(self, mu, sigma_eff, T, vx, vy, ax, ay, kappa, candidates):
        if not USE_L4_BOLTZMANN or not candidates:
            arr = np.array(candidates) if candidates else np.array([mu])
            return float(arr[:, 0].mean()), float(arr[:, 1].mean())

        log_w = []
        for cx, cy in candidates:
            pred_speed = safe_norm(vx, vy)
            disp = safe_norm(cx - mu[0], cy - mu[1])
            E = self.energy((cx, cy), mu, sigma_eff,
                            abs(disp - pred_speed), safe_norm(ax, ay), abs(kappa))
            log_w.append(-E / max(T, EPS))

        log_w = np.array(log_w)
        log_w -= log_w.max()          # log-sum-exp shift
        w = np.exp(log_w)
        total = w.sum()
        if total < EPS:
            arr = np.array(candidates)
            return float(arr[:, 0].mean()), float(arr[:, 1].mean())
        w /= total
        arr = np.array(candidates)
        return float((w * arr[:, 0]).sum()), float((w * arr[:, 1]).sum())

    def detection_energy(self, det_cx, det_cy, mu, sigma_eff,
                         vx, vy, ax, ay, kappa, T):
        disp = safe_norm(det_cx - mu[0], det_cy - mu[1])
        E = self.energy((det_cx, det_cy), mu, sigma_eff,
                        abs(disp - safe_norm(vx, vy)), safe_norm(ax, ay), abs(kappa))
        return E


# ================================================================
# LAYER 5 — Bloch Sphere Engine (singleton)
# ================================================================
class BlochEngine:

    def update(self, vx, vy, ax, ay, kappa, speed):
        if not USE_L5_BLOCH:
            return (0.0,) * 6

        phi_A   = float(np.arctan2(vy, vx + EPS))
        theta_A = float(1.0 - np.exp(-speed / BLOCH_TAU))

        phi_B   = float(np.sign(kappa) * np.pi / 2)
        theta_B = float(abs(kappa) / (abs(kappa) + BLOCH_K + EPS))

        phi_C   = float(np.arctan2(ay, ax + EPS))
        if speed < SPEED_S1:
            theta_C = 0.0
        elif speed < SPEED_S2:
            theta_C = float(np.pi / 2)
        else:
            theta_C = float(np.pi)

        return (theta_A, phi_A, theta_B, phi_B, theta_C, phi_C)

    def amplitudes(self, theta, phi):
        return np.cos(theta / 2), np.exp(1j * phi) * np.sin(theta / 2)

    def label(self, tA, pA, tB, pB, tC, pC):
        d = "uncertain" if tA < 0.3 else f"{np.degrees(pA):.0f}deg"
        t = "straight"  if tB < 0.1 else ("left" if pB > 0 else "right")
        s = "stopped"   if tC < 0.1 else ("gliding" if tC < 2.0 else "burst")
        return f"dir={d} turn={t} spd={s}"


# ================================================================
# LAYER 6 — Tensor Engine (singleton)
# ================================================================
class TensorEngine:

    def __init__(self):
        self.alpha = TENSOR_ALPHA.copy()
        self._bloch = BlochEngine()    # [H4] singleton inside

    def compute(self, bloch_state, kappa, speed, ax, ay, R):
        if not USE_L6_TENSOR:
            return np.ones((2, 2, 2), float)

        tA, pA, tB, pB, tC, pC = bloch_state
        aA, bA = self._bloch.amplitudes(tA, pA)
        aB, bB = self._bloch.amplitudes(tB, pB)
        aC, bC = self._bloch.amplitudes(tC, pC)

        decay = np.exp(
            -TENSOR_LAMBDA1 * abs(kappa)
            -TENSOR_LAMBDA2 * safe_norm(ax, ay)
            -TENSOR_LAMBDA3 * (1.0 / max(R, EPS))
        )

        ampA = [abs(aA)**2, abs(bA)**2]
        ampB = [abs(aB)**2, abs(bB)**2]
        ampC = [abs(aC)**2, abs(bC)**2]

        c = np.empty((2, 2, 2), float)
        for i in range(2):
            for j in range(2):
                for k in range(2):
                    c[i,j,k] = self.alpha[i,j,k] * ampA[i] * ampB[j] * ampC[k] * decay

        norm = np.linalg.norm(c) + EPS
        return c / norm

    def mismatch(self, c1, c2):
        if not USE_L6_TENSOR:
            return 0.0
        return float(np.linalg.norm(c1 - c2))


# ================================================================
# MODULE-LEVEL SINGLETONS — [H4] one instance, not per-cell
# ================================================================
_BLOCH  = BlochEngine()
_TENSOR = TensorEngine()
_BOLTZ  = BoltzmannEngine()
_WAVE   = WavepacketEngine()
_POLY   = PolynomialPredictor()


# ================================================================
# TRACK OBJECT
# ================================================================
class QSortTrack:

    def __init__(self, det, dist_max_px):
        self.id = _next_id()
        self._dist_max = dist_max_px

        det = np.asarray(det, dtype=float)
        self.bbox   = det.copy()             # [M2] always numpy
        self.x      = (det[0] + det[2]) / 2.0
        self.y      = (det[1] + det[3]) / 2.0

        # --- L1 ---
        self.vx = self.vy = 0.0
        self._last_real_vx = 0.0      # F2: last velocity from real detection
        self._last_real_vy = 0.0
        self.ax = self.ay = 0.0
        self.jx = self.jy = 0.0
        self.kappa = 0.0
        self.speed = 0.0
        self.meff  = 1.0
        self.px = self.py = 0.0
        self.sigma_p     = SIGMA_INIT
        self.sigma_theta = 0.0

        # [C2] Single R — no physics overwrite
        self.R = R_INIT

        # --- L2 --- [H1] separate real-detection history
        self.real_x  = deque([self.x], maxlen=HIST_LEN)
        self.real_y  = deque([self.y], maxlen=HIST_LEN)
        self.real_t  = deque([0.0],    maxlen=HIST_LEN)
        self.poly_cx = None
        self.poly_cy = None
        self._poly_t0 = self._poly_t1 = 0.0

        # Full history (real + synthetic) for physics derivatives
        self.hist_x = deque([self.x], maxlen=HIST_LEN)
        self.hist_y = deque([self.y], maxlen=HIST_LEN)
        self.vel_hist = deque(maxlen=HIST_LEN)
        self.acc_hist = deque(maxlen=HIST_LEN)

        # --- L3 ---
        self.mu_x    = self.x
        self.mu_y    = self.y
        self.sigma_w = SIGMA_INIT
        self.phi_wp  = 0.0

        # --- L4 ---
        self.boltz_T = TEMP_INIT

        # --- L5/L6 ---
        self.bloch_state = (0.0,) * 6
        self.tensor      = np.zeros((2, 2, 2), float)

        # Bookkeeping
        self.missed        = 0
        self.age           = 1
        self.frame_idx     = 0
        # [C1] confirmed_hits: once >= BIRTH_CONFIRM, never drops below 1
        self.match_total = 1         # non-decreasing; never reset by predict()
        self._ever_confirmed = False

        self._refresh_physics(real=True)

    # ----------------------------------------------------------------
    # [C2] Physics update — does NOT touch self.R
    # ----------------------------------------------------------------
    def _refresh_physics(self, real=True):
        h = self.hist_x
        if len(h) >= 2:
            vx = (h[-1] - h[-2]) / DT
            vy = (self.hist_y[-1] - self.hist_y[-2]) / DT
        else:
            vx = vy = 0.0

        if self.vel_hist:
            pvx, pvy = self.vel_hist[-1]
            ax = (vx - pvx) / DT
            ay = (vy - pvy) / DT
        else:
            ax = ay = 0.0

        if self.acc_hist:
            pax, pay = self.acc_hist[-1]
            jx = (ax - pax) / DT
            jy = (ay - pay) / DT
        else:
            jx = jy = 0.0

        # [M5] Smooth velocity over last 3 samples for curvature
        if len(self.vel_hist) >= 3:
            vx_s = np.mean([v[0] for v in list(self.vel_hist)[-3:]])
            vy_s = np.mean([v[1] for v in list(self.vel_hist)[-3:]])
            ax_s = np.mean([a[0] for a in list(self.acc_hist)[-3:]]) if len(self.acc_hist) >= 3 else ax
            ay_s = np.mean([a[1] for a in list(self.acc_hist)[-3:]]) if len(self.acc_hist) >= 3 else ay
        else:
            vx_s, vy_s, ax_s, ay_s = vx, vy, ax, ay

        speed = safe_norm(vx_s, vy_s)
        kappa = (abs(vx_s * ay_s - vy_s * ax_s) / (speed**3 + EPS)) if speed > EPS else 0.0

        self.vel_hist.append((vx, vy))
        self.acc_hist.append((ax, ay))

        self.vx, self.vy   = vx, vy
        self.ax, self.ay   = ax, ay
        self.jx, self.jy   = jx, jy
        self.kappa = kappa
        self.speed = speed
        self.meff  = 1.0 + speed
        self.px    = self.meff * vx
        self.py    = self.meff * vy

        # sigma_p from residuals on real history only
        if real and len(self.real_x) >= 3:
            rxs = np.array(self.real_x)[-6:]
            rys = np.array(self.real_y)[-6:]
            vx_e = np.diff(rxs).mean() if len(rxs) >= 2 else 0
            vy_e = np.diff(rys).mean() if len(rys) >= 2 else 0
            exp_x = rxs[0] + vx_e * np.arange(len(rxs))
            exp_y = rys[0] + vy_e * np.arange(len(rys))
            self.sigma_p = max(float(np.std(rxs - exp_x)),
                               float(np.std(rys - exp_y)), SIGMA_MIN)
        elif not real:
            pass   # keep existing sigma_p during occlusion

        if len(self.vel_hist) >= 2:
            angles = [np.arctan2(v[1], v[0] + EPS) for v in list(self.vel_hist)[-6:]]
            self.sigma_theta = float(np.std(angles))

    # ----------------------------------------------------------------
    # [H1] Polynomial — fit on REAL history only
    # ----------------------------------------------------------------
    def _update_poly(self):
        if not USE_L2_POLYNOMIAL or len(self.real_t) < 3:
            return
        win = _POLY.window_size(self.R)
        tx  = np.array(list(self.real_t)[-win:])
        px  = np.array(list(self.real_x)[-win:])
        py  = np.array(list(self.real_y)[-win:])
        t0, t1 = tx[0], tx[-1]
        if t1 > t0:
            tn = (tx - t0) / (t1 - t0 + EPS)
        else:
            tn = np.zeros_like(tx)
        self.poly_cx = _POLY.fit(tn, px, POLY_DEGREE)
        self.poly_cy = _POLY.fit(tn, py, POLY_DEGREE)
        self._poly_t0 = t0
        self._poly_t1 = t1

    def _poly_candidates(self):
        if not USE_L2_POLYNOMIAL or self.poly_cx is None:
            return []
        t0, t1 = self._poly_t0, self._poly_t1
        if t1 <= t0:
            return []
        dt_norm = DT / (t1 - t0 + EPS)
        t_next  = 1.0 + dt_norm
        xp, *_ = _POLY.predict_at(self.poly_cx, t_next)
        yp, *_ = _POLY.predict_at(self.poly_cy, t_next)
        if xp is None:
            return []
        t_next2 = t_next + 0.5 * dt_norm
        xp2, *_ = _POLY.predict_at(self.poly_cx, t_next2)
        yp2, *_ = _POLY.predict_at(self.poly_cy, t_next2)
        # [H6] Fix Python `or` — use explicit None check
        xp2 = xp2 if xp2 is not None else xp
        yp2 = yp2 if yp2 is not None else yp
        return [(xp, yp), (xp2, yp2)]

    # ----------------------------------------------------------------
    # Wavepacket propagation (L3)
    # ----------------------------------------------------------------
    def _propagate_wave(self):
        D = _WAVE.diffusion(self.R, self.kappa)
        # F2: use last real velocity (not degraded synthetic) for mu propagation
        self.mu_x   += self._last_real_vx * DT
        self.mu_y   += self._last_real_vy * DT
        self.sigma_w = _WAVE.spread(self.sigma_w, D)
        self.phi_wp  = float(self.px * self.mu_x + self.py * self.mu_y)

    # ----------------------------------------------------------------
    # Bloch + Tensor update (L5/L6)
    # ----------------------------------------------------------------
    def _update_regime(self):
        self.bloch_state = _BLOCH.update(
            self.vx, self.vy, self.ax, self.ay, self.kappa, self.speed)
        self.tensor = _TENSOR.compute(
            self.bloch_state, self.kappa, self.speed,
            self.ax, self.ay, self.R)

    # ----------------------------------------------------------------
    # PREDICT — no detection this frame
    # ----------------------------------------------------------------
    def predict(self):
        # Physics forward candidates
        cx1 = self.x + self.vx
        cy1 = self.y + self.vy
        cx2 = cx1 + 0.5 * self.ax
        cy2 = cy1 + 0.5 * self.ay
        cx3 = cx2 + (1.0/6.0) * self.jx
        cy3 = cy2 + (1.0/6.0) * self.jy
        cands = [(cx1, cy1), (cx2, cy2), (cx3, cy3)]

        # Curvature deflection only when meaningful
        if self.kappa > 0.01:
            cands.append((self.x - self.kappa * self.vy,
                          self.y + self.kappa * self.vx))

        # Polynomial candidates from real history
        cands += self._poly_candidates()

        # Boltzmann pick
        sigma_eff = max(self.sigma_w, self.sigma_p, 2.0)
        xp, yp = _BOLTZ.best_position(
            (self.x, self.y), sigma_eff, self.boltz_T,
            self.vx, self.vy, self.ax, self.ay, self.kappa, cands)

        # Update full history (synthetic) — not real_hist
        self.frame_idx += 1
        self.hist_x.append(xp)
        self.hist_y.append(yp)
        self.x = xp
        self.y = yp

        self._refresh_physics(real=False)
        # Poly NOT refitted during occlusion (real history unchanged)
        self._propagate_wave()
        self.boltz_T += TEMP_HEAT_RATE
        self._update_regime()

        self.missed += 1
        self.age    += 1
        # [C1] confirmed_hits: clamp to 1 min so confirmed track stays visible
        # match_total is non-decreasing — never decremented by predict()
        pass  # C1: no action needed here
        # [C2] R decays once, here, not in _refresh_physics
        self.R = max(0.0, self.R - R_DECAY)

    # ----------------------------------------------------------------
    # COLLAPSE — detection matched
    # ----------------------------------------------------------------
    def collapse(self, det):
        det = np.asarray(det, dtype=float)
        self.bbox = det.copy()
        xm = (det[0] + det[2]) / 2.0
        ym = (det[1] + det[3]) / 2.0

        # L1 — update both histories
        self.frame_idx += 1
        self.hist_x.append(xm)
        self.hist_y.append(ym)
        self.real_x.append(xm)
        self.real_y.append(ym)
        self.real_t.append(float(self.frame_idx))
        self.x = xm
        self.y = ym

        self._refresh_physics(real=True)
        self._update_poly()          # refit on real history

        # L3 — collapse wavepacket
        self.mu_x, self.mu_y, self.sigma_w, self.phi_wp = \
            _WAVE.collapse_to(xm, ym, self.px, self.py)

        # L4 — reset temperature
        self.boltz_T = TEMP_MIN

        self._update_regime()

        # Bookkeeping
        self.missed = 0
        self.age   += 1
        # [C2] R boosted once, cleanly
        self.R = min(R_MAX, self.R + R_BOOST)
        # F2: snapshot real velocity for wavepacket mu propagation during occlusion
        self._last_real_vx = self.vx
        self._last_real_vy = self.vy
        # [C1] confirmed_hits: increment, mark ever-confirmed
        self.match_total += 1      # C1: only goes up
        if self.match_total >= BIRTH_CONFIRM:
            self._ever_confirmed = True

    # ----------------------------------------------------------------
    # Predicted bbox for cost matrix IoU gate
    # ----------------------------------------------------------------
    def predicted_bbox(self):
        dx = self.mu_x - self.x
        dy = self.mu_y - self.y
        b = self.bbox
        return np.array([b[0]+dx, b[1]+dy, b[2]+dx, b[3]+dy, b[4]])

    # ----------------------------------------------------------------
    # Export row (20 values)
    # ----------------------------------------------------------------
    def export_row(self):
        tA, pA, tB, pB, tC, pC = self.bloch_state
        b = self.bbox
        return [b[0], b[1], b[2], b[3], self.id,
                self.vx, self.vy, self.ax, self.ay, self.jx, self.jy,
                self.kappa, self.px, self.py,
                self.sigma_p, self.sigma_theta, self.R,
                tA, tB, tC]

    def regime_str(self):
        return _BLOCH.label(*self.bloch_state)

    @property
    def is_exported(self):
        """[C1] True once confirmed; stays true even through brief gaps."""
        return self._ever_confirmed and self.missed == 0


# ================================================================
# [C3] COST MATRIX — normalised terms
# ================================================================
def build_cost_matrix(tracks, detections, dist_max_px):
    import numpy as np
    N, M = len(tracks), len(detections)
    if N == 0 or M == 0:
        return np.zeros((N, M), float)
    C = np.full((N, M), 1e6, float)
    det_arr = np.array([[d[0],d[1],d[2],d[3]] for d in detections], float)
    det_cx  = (det_arr[:,0] + det_arr[:,2]) / 2
    det_cy  = (det_arr[:,1] + det_arr[:,3]) / 2
    for i, T in enumerate(tracks):
        dx = T.mu_x - T.x; dy = T.mu_y - T.y
        pb = np.array([T.bbox[0]+dx, T.bbox[1]+dy, T.bbox[2]+dx, T.bbox[3]+dy])
        dists = np.hypot(T.mu_x - det_cx, T.mu_y - det_cy)
        valid = np.where(dists < dist_max_px)[0]
        if not valid.size: continue
        d_v = dists[valid]
        xA=np.maximum(pb[0],det_arr[valid,0]); yA=np.maximum(pb[1],det_arr[valid,1])
        xB=np.minimum(pb[2],det_arr[valid,2]); yB=np.minimum(pb[3],det_arr[valid,3])
        inter=np.maximum(0,xB-xA)*np.maximum(0,yB-yA)
        aA=max((pb[2]-pb[0])*(pb[3]-pb[1]),1e-6)
        aB=np.maximum((det_arr[valid,2]-det_arr[valid,0])*(det_arr[valid,3]-det_arr[valid,1]),1e-6)
        iouv=inter/(aA+aB-inter+EPS)
        ok=np.where(iouv>=IOU_MIN)[0]
        if not ok.size: continue
        js=valid[ok]; d_ok=d_v[ok]; iou_ok=iouv[ok]
        cx_ok=det_cx[js]; cy_ok=det_cy[js]
        c_dist=np.minimum(d_ok/(dist_max_px+EPS),1.0)
        if USE_L4_BOLTZMANN:
            se=max(T.sigma_w,T.sigma_p,2.0)
            E=d_ok**2/(2*se**2+EPS)+BOLTZ_BETA*np.abs(d_ok-T.speed)+BOLTZ_ALPHA*np.hypot(T.ax,T.ay)+BOLTZ_GAMMA*abs(T.kappa)
            c_boltz=1.0-np.exp(-E/max(T.boltz_T,EPS))
        else: c_boltz=0.0
        if T.speed>EPS:
            cos_a=((cx_ok-T.x)*T.vx+(cy_ok-T.y)*T.vy)/(T.speed*d_ok+EPS)
            c_dir=(1.0-np.abs(cos_a))/2.0
        else: c_dir=0.0
        if USE_L6_TENSOR:
            c_tensor=np.empty(len(js))
            for k,(j_idx,spd) in enumerate(zip(js,d_ok)):
                db=_BLOCH.update(float(cx_ok[k]-T.x),float(cy_ok[k]-T.y),0,0,0,float(spd))
                dt=_TENSOR.compute(db,0,float(spd),0,0,T.R)
                c_tensor[k]=_TENSOR.mismatch(T.tensor,dt)/(np.sqrt(8)+EPS)
        else: c_tensor=0.0
        C[i,js]=(COST_DIST_W*c_dist+COST_BOLTZ_W*c_boltz+COST_DIR_W*c_dir+COST_TENSOR_W*c_tensor+COST_IOU_W*(1.0-iou_ok))
    return C


# ================================================================
# RE-ID GATE — [M1] uses mu_x (predicted), not stale x
# ================================================================
def allow_reid(T, det, dist_max_px):
    if T.missed > MAX_MISSED:
        return False
    cx = (det[0] + det[2]) / 2.0
    cy = (det[1] + det[3]) / 2.0
    search_r = max(0.30 * dist_max_px, T.sigma_w * 8)   # F3: wider search
    if safe_norm(cx - T.mu_x, cy - T.mu_y) > search_r:  # [M1]
        return False
    if T.R < 0.15:
        return False
    return True


# ================================================================
# MAIN TRACKER
# ================================================================
class QSortPhysicsTracker:
    """
    Call:  tracker = QSortPhysicsTracker(frame_w=1920, frame_h=1080)
           out = tracker.update(dets)   # dets: list of [x1,y1,x2,y2,conf]
    Returns np.array (N, 20).
    """

    def __init__(self, frame_w=1920, frame_h=1080):
        # [L2] Resolution-aware distance gate
        diag = np.sqrt(frame_w**2 + frame_h**2)
        self.dist_max_px = float(diag * BOLTZ_DIST_FRAC)
        self.tracks: list[QSortTrack] = []
        _D(f"QSort init: frame={frame_w}x{frame_h} "
           f"dist_max={self.dist_max_px:.0f}px")

    # ----------------------------------------------------------------
    def update(self, dets):
        # [M6] Filter by confidence
        dets = [np.asarray(d, dtype=float) for d in dets
                if len(d) >= 5 and d[4] >= MIN_CONF]

        _D(f"FRAME tracks={len(self.tracks)} dets={len(dets)}")

        if not self.tracks:
            for d in dets:
                t = QSortTrack(d, self.dist_max_px)
                self.tracks.append(t)
            return self._export()

        if not dets:
            for t in self.tracks:
                t.predict()
            self._cleanup()
            return self._export()

        C = build_cost_matrix(self.tracks, dets, self.dist_max_px)
        rows, cols = linear_sum_assignment(C)

        assigned_t, assigned_d = set(), set()
        for r, c in zip(rows, cols):
            if C[r, c] < 1.0:           # normalised — 1.0 is the ceiling
                self.tracks[r].collapse(dets[c])
                assigned_t.add(r)
                assigned_d.add(c)
                _D(f"ASSIGN id={self.tracks[r].id} cost={C[r,c]:.3f} "
                   f"{self.tracks[r].regime_str()}")

        for idx, t in enumerate(self.tracks):
            if idx not in assigned_t:
                t.predict()

        for j, d in enumerate(dets):
            if j in assigned_d:
                continue
            # [M1] Best-cost re-ID using mu_x
            best_cost, best_t = 1e9, None
            for t in self.tracks:
                if t.missed > 0 and allow_reid(t, d, self.dist_max_px):
                    cx = (d[0]+d[2]) / 2.0
                    cy = (d[1]+d[3]) / 2.0
                    cost = safe_norm(cx-t.mu_x, cy-t.mu_y) + (1-t.R)*20
                    if cost < best_cost:
                        best_cost, best_t = cost, t
            if best_t is not None:
                best_t.collapse(d)
                _D(f"REID id={best_t.id}")
            else:
                nt = QSortTrack(d, self.dist_max_px)
                self.tracks.append(nt)
                _D(f"BIRTH id={nt.id}")

        self._cleanup()
        return self._export()

    # ----------------------------------------------------------------
    def _cleanup(self):
        def keep(t):
            if t.missed > MAX_MISSED:
                return False
            # R floor only kills unconfirmed (ghost) tracks quickly
            # Confirmed tracks survive until MAX_MISSED regardless of R
            if (not t._ever_confirmed and
                    t.age >= MIN_AGE_DELETE and t.missed > 0 and t.R < R_FLOOR_ACTIVE):
                return False
            return True
        before = len(self.tracks)
        self.tracks = [t for t in self.tracks if keep(t)]
        if len(self.tracks) < before:
            _D(f"CLEANUP removed {before - len(self.tracks)}")

    def _export(self):
        rows = [t.export_row() for t in self.tracks if t.is_exported]
        return np.array(rows, float) if rows else np.empty((0, 20), float)


# ================================================================
# ABLATION RUNNER — [L3] reproduce paper Table 3
# ================================================================
def run_ablation(frame_sequences: dict, frame_w=1920, frame_h=1080):
    """
    frame_sequences: dict of {scenario_name: list_of_det_lists}
    Runs all 32 layer combinations and prints a comparison table.
    Returns dict of results.
    """
    import itertools
    global USE_L2_POLYNOMIAL, USE_L3_WAVEPACKET, USE_L4_BOLTZMANN
    global USE_L5_BLOCH, USE_L6_TENSOR

    flags = ['L2','L3','L4','L5','L6']
    originals = [USE_L2_POLYNOMIAL, USE_L3_WAVEPACKET,
                 USE_L4_BOLTZMANN, USE_L5_BLOCH, USE_L6_TENSOR]
    results = {}

    print(f"{'Config':<32} " +
          " ".join(f"{s[:12]:>12}" for s in frame_sequences))
    print("-" * (32 + 13 * len(frame_sequences)))

    for combo in itertools.product([False, True], repeat=5):
        USE_L2_POLYNOMIAL = combo[0]
        USE_L3_WAVEPACKET = combo[1]
        USE_L4_BOLTZMANN  = combo[2]
        USE_L5_BLOCH      = combo[3]
        USE_L6_TENSOR     = combo[4]

        label = "+".join(f for f, on in zip(flags, combo) if on) or "BASE"
        row_results = {}

        for name, frames in frame_sequences.items():
            reset_id_counter()
            tracker = QSortPhysicsTracker(frame_w, frame_h)
            births = set()
            for dets in frames:
                out = tracker.update(dets)
                for r in out:
                    births.add(int(r[4]))
            row_results[name] = len(births)

        results[label] = row_results
        print(f"{label:<32} " +
              " ".join(f"{row_results[s]:>12}" for s in frame_sequences))

    # Restore originals
    USE_L2_POLYNOMIAL, USE_L3_WAVEPACKET, USE_L4_BOLTZMANN, \
        USE_L5_BLOCH, USE_L6_TENSOR = originals
    return results


# ================================================================
# SMOKE TEST
# ================================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG if DEBUG_LOG else logging.INFO)
    print("QSort v2 — smoke test")
    print(f"  L2={USE_L2_POLYNOMIAL} L3={USE_L3_WAVEPACKET} "
          f"L4={USE_L4_BOLTZMANN} L5={USE_L5_BLOCH} L6={USE_L6_TENSOR}")

    tracker = QSortPhysicsTracker(frame_w=1920, frame_h=1080)
    print(f"  dist_max_px = {tracker.dist_max_px:.0f}px")

    np.random.seed(42)
    for frame in range(25):
        if frame < 10:
            cx, cy = 100 + frame*14, 200 + frame*6
        elif frame < 11:
            cx, cy = 100 + 10*14, 200 + 10*6   # brief gap
            out = tracker.update([])
            continue
        else:
            cx, cy = 100 + 10*14 + (frame-11)*5, 200 + 10*6 - (frame-11)*12

        cx += np.random.randn() * 2
        cy += np.random.randn() * 2
        det = [cx-25, cy-20, cx+25, cy+20, 0.94]
        out = tracker.update([det])

        if out.shape[0] > 0:
            r = out[0]
            print(f"  f={frame:2d} id={int(r[4])} "
                  f"pos=({(r[0]+r[2])/2:.0f},{(r[1]+r[3])/2:.0f}) "
                  f"v=({r[5]:.1f},{r[6]:.1f}) "
                  f"kappa={r[11]:.3f} R={r[16]:.3f} "
                  f"regime={tracker.tracks[0].regime_str() if tracker.tracks else '-'}")

    print("\nAll done.")


# FIXES on this version from as compared to previous versions, 
#   [C1] confirmed_hits oscillation — once confirmed, stays confirmed
#   [C2] Double R modification — unified single R system
#   [C3] Cost terms normalised — all terms in [0,1], weights meaningful
#   [H1] Polynomial fit on real-only history (separate real_hist)
#   [H2] Bloch speed thresholds scaled for real fish video
#   [H3] BOLTZ_DIST_MAX derived from frame diagonal, not hardcoded px
#   [H4] BlochEngine + TensorEngine singletons, not per-cell instances
#   [H5] DEBUG_LOG=False by default, uses logging not open/close per call
#   [H6] Python `or` bug fixed in _poly_predict_next
#   [M1] Re-ID uses mu_x (predicted) not stale x
#   [M2] bbox always stored as numpy array
#   [M3] qs_safe_norm alias removed
#   [M4] _id_counter is thread-safe with a lock
#   [M5] Curvature smoothed over 3-frame velocity window
#   [M6] MIN_CONF threshold filters low-confidence detections
#   [L1] TensorEngine alpha values exposed as tunable parameters
#   [L2] Frame size passed to tracker init for resolution-aware thresholds
#   [L3] Ablation runner included as run_ablation()
# ================================================================
