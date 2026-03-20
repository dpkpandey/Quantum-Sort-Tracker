# QSort Physics Tracker — Level 4A optimesed version 
This tracker is most recent one  and optimesed for level 4 ( it includes all level as you want to impliment)

> **QuantumSort (QSort):** A Classical–Quantum Hybrid Framework for Nonlinear Motion Tracking  
> **Author:** Deepak Pandey · Australia · ORCID: [0009-0006-5313-0222](https://orcid.org/0009-0006-5313-0222)  
> **License:** Free to use with citation · February 2026

---

## Table of Contents

- [What Is QSort?](#what-is-qsort)
- [Design Motivation](#design-motivation)
- [Theoretical Foundations](#theoretical-foundations)
  - [1. Classical Motion State Vector (QPCMSV)](#1-classical-motion-state-vector-qpcmsv)
  - [2. Boltzmann Spatial Probability Field](#2-boltzmann-spatial-probability-field)
  - [3. Quantum-Inspired Wavepacket Dynamics](#3-quantum-inspired-wavepacket-dynamics)
  - [4. Bloch-Sphere Motion Encoding](#4-bloch-sphere-motion-encoding)
  - [5. Multi-Qubit Motion Tensor](#5-multi-qubit-motion-tensor)
- [How the Tracker Works — Step by Step](#how-the-tracker-works--step-by-step)
  - [PhysicsTrack Object](#physicstrack-object)
  - [BoltzmannPredictor](#boltzmannpredictor)
  - [Cost Matrix and Hungarian Assignment](#cost-matrix-and-hungarian-assignment)
  - [Re-ID and Recovery Logic](#re-id-and-recovery-logic)
  - [Track Lifecycle](#track-lifecycle)
- [Output Format](#output-format)
- [Parameters and Tuning Guide](#parameters-and-tuning-guide)
  - [Global Constants](#global-constants)
  - [Cost Function Weights](#cost-function-weights)
  - [Reliability Score (R) Weights](#reliability-score-r-weights)
  - [Boltzmann Energy Weights](#boltzmann-energy-weights)
  - [Re-ID Thresholds](#re-id-thresholds)
  - [Tracker Constructor Parameters](#tracker-constructor-parameters)
- [Quick Start](#quick-start)
- [Integration with YOLO](#integration-with-yolo)
- [Debug Logging](#debug-logging)
- [Citation](#citation)
- [Roadmap](#roadmap)

---

## What Is QSort?

**QSort (QuantumSort)** is a multi-object tracker that replaces the standard linear Kalman prediction model with a layered physics engine inspired by:

- **Classical mechanics** — position, velocity, acceleration, jerk
- **Statistical mechanics** — Boltzmann energy-based spatial probability fields
- **Quantum mechanics** — wavepacket uncertainty evolution and Bloch-sphere regime encoding

It was originally developed for fish tracking in aquaculture video analytics (Humpty Doo Barramundi R&D), where conventional trackers fail due to turbulence, curved trajectories, and sudden accelerations. The same framework directly applies to any system where objects move nonlinearly: molecular dynamics, drone tracking, crowd analytics, sports biomechanics, and robotics.

This repository contains **Level 4A (Debug2)** — the most instrumented build, with full per-frame logging of every assignment decision, birth, death, recovery, and physics prediction.

---

## Design Motivation

QSort was designed to handle motion scenarios where linear-assumption trackers tend to struggle — specifically turbulent aquatic environments with curved trajectories, sudden accelerations, and frequent occlusions. The design choices (jerk, curvature, Boltzmann weighting, wavepacket-style uncertainty) each address a specific physical challenge observed during development.

> **Note:** Formal benchmarking against other trackers (SORT, DeepSORT, ByteTrack, etc.) is ongoing. The claims above are based on design intent and observed behaviour during development — not yet on published comparative metrics. Contributions and test results are welcome.

---

## Theoretical Foundations

### 1. Classical Motion State Vector (QPCMSV)

QSort tracks each object with a full **QPand Classical Motion State Vector**:

```
State = [x, y, vx, vy, ax, ay, jx, jy, curvature, σ_x, σ_y, σ_θ, m_eff, px, py, R]
```

| Symbol | Meaning |
|---|---|
| `x, y` | Bounding box centre |
| `vx, vy` | Velocity (finite difference, 1-frame DT) |
| `ax, ay` | Acceleration |
| `jx, jy` | Jerk (rate of change of acceleration) |
| `curvature` | `|vx·ay − vy·ax| / speed³` — how sharply the object is turning |
| `σ_x, σ_y` | Positional uncertainty (std dev over history window) |
| `σ_θ` | Angular uncertainty (std dev of heading angles) |
| `m_eff` | Effective mass = `1 + speed` (simple model) |
| `px, py` | Momentum = `m_eff · v` |
| `R` | Reliability score ∈ [0, 1] |

This extended state vector is what distinguishes QSort from trackers that carry only `[x, y, vx, vy]`.

---

### 2. Boltzmann Spatial Probability Field

Instead of a Kalman covariance ellipse, QSort uses a **Boltzmann-weighted probability distribution** over candidate next positions.

**Energy function** for an object state T:

```
E(T) = 0.4·|v| + 0.7·|a| + 1.2·|j| + 1.8·κ + 1.5·(1−R) + 0.5·(σ_x+σ_y) + 0.3·σ_θ
```

Higher energy = more chaotic / uncertain motion = wider spatial spread.

**Effective temperature** (controls spread):

```
T_temp = base_temp · (1 + σ_x + σ_y + 0.5·σ_θ + (1−R))
```

**Boltzmann weight** for each candidate position:

```
w_i = exp(−E / T_temp)
```

The **predicted position** is the weighted expectation over 4 kinematic candidates (velocity-only, velocity+acceleration, velocity+acc+jerk, curvature-corrected). This is analogous to the Boltzmann distribution in statistical mechanics — high-energy, uncertain states spread their probability over a wider region of space.

---

### 3. Quantum-Inspired Wavepacket Dynamics

Each track carries an implicit **wavepacket** centred on its predicted position. The wavepacket width (uncertainty) evolves between frames:

- **Spreads** when no detection is found (missed frames), analogous to free quantum propagation
- **Collapses** when a detection is assigned, analogous to wavefunction collapse upon measurement

The positional uncertainties `σ_x` and `σ_y` play the role of wavepacket width. The angular uncertainty `σ_θ` encodes the directional coherence of the wave. This is a classical analogue of Heisenberg-style uncertainty — as we become less sure of the object's state, the spatial probability field broadens.

---

### 4. Bloch-Sphere Motion Encoding

QSort conceptually encodes each object's motion regime as three coupled **qubits** on Bloch spheres:

| Sphere | Encodes |
|---|---|
| **A — Direction Qubit** | Current heading angle relative to history mean |
| **B — Turning Qubit** | Rate of direction change (curvature) |
| **C — Speed-Regime Qubit** | Fast / slow / stopped regime |

These three spheres are **classically coupled**: a fast-turning object in sphere B influences the position uncertainty in sphere A, etc. Regime switches (e.g., fish suddenly reversing direction) are represented as rotations on sphere C.

In this Level 4A implementation, the Bloch encoding is reflected through the composite reliability score `R` and the curvature/jerk penalisation rather than as explicit qubit objects.

---

### 5. Multi-Qubit Motion Tensor

The full motion state is the **tensor product** of the three qubit states:

```
|Ψ⟩ = |Direction⟩ ⊗ |Turning⟩ ⊗ |Speed-Regime⟩
```

This 8-dimensional state manifold allows QSort to represent multiple simultaneous motion regimes. Classical analogues of quantum gates (rotation operators) drive transitions between states. In Level 4A, this manifests through the jerk and curvature terms in the cost and reliability functions.

---

## How the Tracker Works — Step by Step

### PhysicsTrack Object

Each active object is a `PhysicsTrack` instance. On every matched frame:

1. **`update_bbox(det)`** — extracts bounding box and centre `(x, y)`
2. **`update_state()`** — appends to rolling history and computes:
   - Finite-difference velocity → acceleration → jerk
   - Curvature from cross product: `|v × a| / |v|³`
   - Positional and angular std dev over the last `HIST_LEN` frames
   - Effective mass and momentum
   - Reliability score `R = exp(−penalty)` where penalty sums jerk, curvature, and uncertainty contributions
3. **`predict_candidates()`** — generates 4 physically motivated next positions:
   - `C1`: velocity only
   - `C2`: velocity + ½·acceleration
   - `C3`: C2 + ⅙·jerk
   - `C4`: curvature-corrected lateral displacement

### BoltzmannPredictor

Given a `PhysicsTrack`, the predictor:

1. Computes `E` (energy) from jerk, curvature, uncertainty, and `(1−R)`
2. Computes `T_temp` (temperature) from uncertainty and `(1−R)`
3. Assigns Boltzmann weights `w = exp(−E/T_temp)` to all 4 candidates
4. Returns the weighted mean `(xp, yp)` as the predicted position

### Cost Matrix and Hungarian Assignment

For N tracks and M detections, a cost matrix `C[N×M]` is built:

```
C[i,j] = 0.4 · dist(predicted_i, det_j)
        + 80 · (1 − IoU(track_i_bbox, det_j_bbox))
        + 1.1 · physics_mismatch(track_i, det_j)
```

Where `physics_mismatch` penalises:
- Displacement deviation from expected velocity
- Angle mismatch between motion direction and displacement vector
- Acceleration and jerk magnitude
- Curvature and uncertainty
- Low reliability `(1 − R)`

Any pair with `dist > 350px` or `IoU < 0.01` is hard-blocked (`C = 1e6`).

The **Hungarian algorithm** (`scipy.optimize.linear_sum_assignment`) finds the globally optimal minimum-cost assignment.

### Re-ID and Recovery Logic

Before spawning a new track for an unmatched detection, QSort attempts **re-identification** against missed tracks:

```python
allow_reid(T, det):
    T.missed <= 5        # not vanished too long
    displacement <= 100px
    T.R >= 0.3           # track was reliable before disappearing
```

If re-ID succeeds, the same track ID is restored. No ID is ever reused for a *different* object — ID uniqueness is globally guaranteed via a monotonically incrementing `next_id` counter.

### Track Lifecycle

```
Detection → Birth (new ID)
         ↓
    update() each frame it is matched
         ↓
    mark_missed() when not matched (up to max_missed frames)
         ↓
    allow_reid() attempt on reappearance (within reid_gap frames)
         ↓
    _cleanup() removes tracks with missed > max_missed
```

---

## Output Format

`tracker.update(dets)` returns a NumPy array of shape `(N, 18)`:

| Index | Field | Description |
|---|---|---|
| 0–3 | `x1, y1, x2, y2` | Bounding box |
| 4 | `id` | Unique track ID (never reused) |
| 5–6 | `vx, vy` | Velocity (px/frame) |
| 7–8 | `ax, ay` | Acceleration (px/frame²) |
| 9–10 | `jx, jy` | Jerk (px/frame³) |
| 11 | `curvature` | Path curvature (1/px) |
| 12–13 | `px, py` | Momentum = m_eff · v |
| 14–15 | `sigma_x, sigma_y` | Positional uncertainty (px) |
| 16 | `sigma_theta` | Angular uncertainty (radians) |
| 17 | `R` | Reliability ∈ [0, 1] |

Only tracks with `missed == 0` (actively matched this frame) are exported.

---

## Parameters and Tuning Guide

### Global Constants

```python
DT = 1.0        # Time step between frames (set to 1/fps for real-time scaling)
HIST_LEN = 15   # Rolling history window for physics computation
EPS = 1e-6      # Numerical stability floor
```

**Tuning advice:**
- Increase `HIST_LEN` (e.g., 20–30) for smoother physics in slow-moving scenes; decrease (e.g., 5–8) for fast, chaotic motion where old history misleads predictions.
- Set `DT = 1.0/fps` (e.g., `1/30` for 30fps video) if you want velocity in px/sec rather than px/frame.

---

### Cost Function Weights

In `build_cost_matrix`:

```python
C[i,j] = 0.4 * dist       # Euclidean distance weight
        + 80 * (1 - iou)   # IoU mismatch weight
        + 1.1 * phys_cost  # Physics mismatch weight
```

| Parameter | Default | When to Increase | When to Decrease |
|---|---|---|---|
| Distance weight `0.4` | `0.4` | Sparse scenes, large gaps | Dense scenes, small objects |
| IoU weight `80` | `80` | Objects with stable sizes | Objects that resize or deform |
| Physics weight `1.1` | `1.1` | Need strict physical plausibility | Objects with very erratic motion |
| `dist > 350` hard block | `350px` | High-res video, fast objects | Low-res or slow scenes |
| `IoU < 0.01` hard block | `0.01` | Mostly keep default | — |

In `physics_mismatch`:

```python
cost = 0.3*vel_dev + 1.0*angle_diff + 0.6*acc_pen + 1.0*jerk_pen
     + 1.2*curvature + 0.4*(sigma_x+sigma_y) + 0.6*(1−R)
```

- **`angle_diff` weight (1.0):** Most important for direction-sensitive scenes (fish, drones). Lower to `0.3–0.5` if objects frequently reverse direction (e.g., brownian motion).
- **`jerk_pen` weight (1.0):** Lower to `0.5` for highly erratic motion (insects, particles). Raise to `2.0` for smooth trajectories (vehicles).
- **`curvature` weight (1.2):** Lower to `0.5` for objects that spiral or make tight turns.

---

### Reliability Score (R) Weights

In `PhysicsTrack.update_state`:

```python
penalty = 0.5*jmag + 0.4*curvature + 0.3*sigma_x + 0.3*sigma_y + 0.2*sigma_theta
R = exp(-penalty)
```

| Parameter | Effect |
|---|---|
| Jerk weight `0.5` | Higher = `R` drops faster on sudden accelerations |
| Curvature weight `0.4` | Higher = tight turns reduce reliability more |
| `sigma_x/y` weights `0.3` | Higher = positional spread degrades reliability more |
| `sigma_theta` weight `0.2` | Higher = directional variance degrades reliability more |

**Tuning advice:** If `R` is collapsing too fast (tracks dying prematurely), reduce all weights by ~30%. If `R` stays too high on clearly bad tracks, increase jerk/curvature weights.

---

### Boltzmann Energy Weights

In `BoltzmannPredictor.compute_energy`:

```python
E = 0.4*|v| + 0.7*|a| + 1.2*|j| + 1.8*κ + 1.5*(1−R) + 0.5*(σ_x+σ_y) + 0.3*σ_θ
```

In `BoltzmannPredictor.compute_temperature`:

```python
T_temp = base_temp * (1 + σ_x + σ_y + 0.5*σ_θ + (1−R))
base_temp = 1.0   # constructor default
```

| Parameter | Effect |
|---|---|
| `base_temp` | Global scale of prediction spread. Raise (e.g., 2.0–5.0) for very unpredictable motion; lower (0.3–0.5) for smooth motion |
| Jerk weight `1.2` | Dominant driver of energy in erratic motion |
| Curvature weight `1.8` | Dominant for spiraling/turning objects |
| `(1−R)` weight `1.5` | Penalises unreliable tracks heavily |

---

### Re-ID Thresholds

In `allow_reid`:

```python
T.missed <= 5     # Max frames a track can be invisible before reid is refused
displacement <= 100px  # Max pixels the object can have moved while hidden
T.R >= 0.3        # Min reliability required for a track to be recoverable
```

| Parameter | Raise when... | Lower when... |
|---|---|---|
| `missed <= 5` | Objects frequently occlude for longer periods | Short occlusions only; want strict matching |
| `displacement <= 100px` | Fast objects, high FPS | Slow objects or low FPS |
| `T.R >= 0.3` | Want aggressive recovery | Want conservative recovery (fewer false re-IDs) |

---

### Tracker Constructor Parameters

```python
QSortPhysicsTracker(max_missed=8, reid_gap=5)
```

| Parameter | Default | Description |
|---|---|---|
| `max_missed` | `8` | Frames a track survives without a match before deletion |
| `reid_gap` | `5` | Reserved for future use (currently re-ID controlled by `allow_reid`) |

**Tuning advice:**
- `max_missed = 8` is good for 25–30 fps video. Scale proportionally to FPS: for 60fps, consider 15–20; for 10fps, use 4–5.
- For high-occlusion scenes (crowded tanks, overlapping paths), raise `max_missed` to 12–15.

---

## Quick Start

```python
import numpy as np
from qsort_tracker import QSortPhysicsTracker

tracker = QSortPhysicsTracker(max_missed=8)

# dets: list of [x1, y1, x2, y2, confidence] per frame
frame_detections = [
    [100, 200, 150, 250, 0.9],
    [300, 100, 360, 155, 0.85],
]

result = tracker.update(np.array(frame_detections))

# result shape: (N, 18)
# Columns: x1 y1 x2 y2 | id | vx vy | ax ay | jx jy | curvature | px py | sx sy stheta | R
for row in result:
    x1, y1, x2, y2, tid = row[:5].astype(int)
    vx, vy = row[5], row[6]
    R = row[17]
    print(f"ID={tid}  bbox=({x1},{y1},{x2},{y2})  v=({vx:.1f},{vy:.1f})  R={R:.3f}")
```

---

## Integration with YOLO

```python
from ultralytics import YOLO
import cv2
import numpy as np
from qsort_tracker import QSortPhysicsTracker

model = YOLO("yolov8n.pt")
tracker = QSortPhysicsTracker(max_missed=8)
cap = cv2.VideoCapture("video.mp4")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, verbose=False)[0]
    boxes = results.boxes

    dets = []
    for box, conf in zip(boxes.xyxy.cpu().numpy(), boxes.conf.cpu().numpy()):
        dets.append([*box, float(conf)])

    tracks = tracker.update(np.array(dets) if dets else np.empty((0, 5)))

    for row in tracks:
        x1, y1, x2, y2, tid = int(row[0]), int(row[1]), int(row[2]), int(row[3]), int(row[4])
        R = row[17]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"ID:{tid} R:{R:.2f}", (x1, y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv2.imshow("QSort", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## Debug Logging

All events are written to `qsort_debug_log.txt`, overwritten each run. Log entries include:

| Tag | Meaning |
|---|---|
| `[BIRTH]` | New track spawned |
| `[ASSIGN]` | Track matched to detection with cost value |
| `[ASSIGN FAIL]` | Hungarian match rejected (cost > 1e5) |
| `[MISS]` | Track not matched; missed counter incremented |
| `[REID]` | Track recovered via re-identification |
| `[NEW]` | New track born from unmatched detection |
| `[DELETE]` | Track purged (exceeded `max_missed`) |

To disable logging in production, replace the `D()` function body with `pass`.

---

## Citation

If you use QSort in your research or product, please cite:

```bibtex
@monograph{pandey2026qsort,
  title   = {QuantumSort (QSort): A Classical–Quantum Hybrid Framework for Nonlinear Motion Tracking},
  author  = {Deepak Pandey},
  year    = {2026},
  month   = {February},
  note    = {Dense Research Monograph. Physics + Machine Learning + Biological Systems},
  orcid   = {0009-0006-5313-0222}
}
```

---

## Roadmap

- [ ] Full Bloch-sphere qubit objects with explicit state vectors
- [ ] Wavepacket width as a standalone object (explicit `σ` evolution equations)
- [ ] Polynomial regression trajectory prediction (degree-configurable)
- [ ] Multi-qubit tensor product state with classical entanglement coupling
- [ ] GPU-accelerated cost matrix via CUDA/CuPy
- [ ] Count logic module (companion to tracker)
- [ ] MOT benchmark evaluation script (HOTA, MOTA, IDF1)
- [ ] Real-time multi-threaded pipeline for aquaculture deployment

---

*This software is distributed freely. Anyone is welcome to use and implement it in their project by citing this work.*  
*— Deepak Pandey, March 2026*
