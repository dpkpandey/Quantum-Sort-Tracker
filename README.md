# QSort Physics Tracker

**Author:** Deepak Pandey  
**Version:** v2 (Full 6-Layer Implementation)

A multi-object tracker that fuses classical physics kinematics with concepts borrowed from quantum mechanics — wavepacket spreading, Bloch-sphere regime encoding, and multi-qubit tensor coupling — to achieve robust tracking under occlusion, crowding, and erratic motion. Originally developed and tested on fish tracking video; designed to be domain-agnostic.

> **Note from the author:** Most of the tracking system is still experimental and largely untested on diverse datasets. The quantum-inspired components do not require quantum hardware — they borrow the mathematical formalism (Bloch spheres, tensor products) as a representational framework. If you want to build on this, a basic understanding of classical mechanics and quantum mechanics notation helps.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Layer Descriptions](#layer-descriptions)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Fine-Tuning Parameters](#fine-tuning-parameters)
- [API Reference](#api-reference)
- [Output Format](#output-format)
- [Ablation Testing](#ablation-testing)
- [Changelog from Previous Version](#changelog-from-previous-version)
- [Known Limitations](#known-limitations)
- [Contributing](#contributing)

---

## Architecture Overview

```
Detection Input  →  [Confidence Gate MIN_CONF]
                          ↓
              ┌───────────────────────────┐
              │     L1: QPCMSV            │  16D classical state vector
              │  (pos, vel, acc, jerk,    │
              │   momentum, curvature)    │
              └────────────┬──────────────┘
                           │
          ┌────────────────┼─────────────────┐
          ↓                ↓                 ↓
  L2: Polynomial    L3: Wavepacket    L4: Boltzmann
  Regression        Spreading         Spatial Field
  (nonlinear        (uncertainty      (temperature-
   prediction)       + collapse)       weighted pick)
          └────────────────┬─────────────────┘
                           ↓
               L5: Bloch-Sphere Encoding
               (3 qubits: direction, turn, speed)
                           ↓
               L6: Multi-Qubit Motion Tensor
               (8D regime fusion + cost matrix)
                           ↓
              Hungarian Assignment → Track Update
```

---

## Layer Descriptions

### L1 — QPCMSV: Extended Classical State Vector (16D)

Each track maintains a 16-dimensional state:

| Component | Variables |
|-----------|-----------|
| Position  | `x, y` |
| Velocity  | `vx, vy` |
| Acceleration | `ax, ay` |
| Jerk | `jx, jy` |
| Curvature | `kappa` |
| Momentum (effective) | `px, py` |
| Uncertainty | `sigma_p, sigma_theta` |
| Reliability | `R` |
| Bloch angles | `theta_A, theta_B, theta_C` |

Curvature `kappa = |vx·ay − vy·ax| / speed³` captures how sharply an object is turning. Effective mass `meff = 1 + speed` scales momentum to penalise fast objects changing direction abruptly.

---

### L2 — Polynomial Regression Predictor

Fits a degree-3 ridge-regularised polynomial to the **real detection history only** (not synthetic/predicted positions during occlusion). Window size adapts with reliability `R`:

| R range | Window |
|---------|--------|
| > 0.7   | 12 frames |
| 0.4–0.7 | 7 frames  |
| < 0.4   | 4 frames  |

Produces up to two candidate future positions fed into the Boltzmann selector.

**Key flag:** `USE_L2_POLYNOMIAL = True`

---

### L3 — Wavepacket Uncertainty Engine

Models positional uncertainty as a Gaussian wavepacket that **spreads during occlusion** and **collapses on detection**:

- Diffusion rate: `D = DIFFUSION_BASE × (2 − R) + DIFFUSION_CURV × kappa`
- Spreading: `σ_new = sqrt(σ² + D·dt)`
- Collapse: on matched detection, `σ → SIGMA_MIN` and phase `φ` is updated

During occlusion, mu propagates using the **last real velocity** (not degraded synthetic velocities), preventing drift compounding.

**Key flag:** `USE_L3_WAVEPACKET = True`

---

### L4 — Boltzmann Spatial Probability Field

Rather than picking the geometrically closest candidate, a Boltzmann-weighted energy function selects the most physically consistent position:

```
E = displacement²/(2σ²) + β·Δv + α·|a| + γ·|κ|
weight ∝ exp(−E / T)
```

Temperature `T` heats up during missed frames (increasing tolerance) and resets to `TEMP_MIN` on detection (tightening the field). This naturally handles re-association after occlusion.

**Key flag:** `USE_L4_BOLTZMANN = True`

---

### L5 — Bloch-Sphere Regime Encoding (3 Qubits)

Motion state is encoded onto three Bloch spheres:

| Qubit | Encodes | θ range | φ range |
|-------|---------|---------|---------|
| A | Direction of travel | 0 (stationary) → π (fast) | Heading angle |
| B | Turn sharpness | 0 (straight) → π (sharp) | Left / Right |
| C | Speed regime | 0 (stopped) → π (burst) | Accel direction |

Speed thresholds scaled for real video (pixels/frame):
- `SPEED_S1 = 2.0` px/frame — stopped → gliding
- `SPEED_S2 = 15.0` px/frame — gliding → burst

**Key flag:** `USE_L5_BLOCH = True`

---

### L6 — Multi-Qubit Motion Tensor (8D Regime Fusion)

Combines the three qubit amplitudes into a `2×2×2` coupling tensor:

```python
c[i, j, k] = α[i,j,k] · |A_i|² · |B_j|² · |C_k|² · decay
```

where `decay = exp(−λ₁|κ| − λ₂|a| − λ₃/R)` penalises unstable states.

The tensor mismatch between track and candidate detection is used as a cost term in the assignment matrix, rewarding motions that are physically consistent with the track's current regime.

Alpha coupling values are tunable:

```python
TENSOR_ALPHA[1, 1, 0] = 1.2   # direction + turn
TENSOR_ALPHA[0, 1, 1] = 1.2   # turn + burst
TENSOR_ALPHA[1, 0, 1] = 1.0   # direction + burst
TENSOR_ALPHA[1, 1, 1] = 1.5   # all three (strongest)
```

**Key flag:** `USE_L6_TENSOR = True`

---

## Installation

```bash
pip install numpy scipy
```

No other dependencies required. No quantum hardware needed.

Python 3.9+ recommended.

---

## Quick Start

```python
from qsort_tracker import QSortPhysicsTracker

# Initialise once per video sequence
tracker = QSortPhysicsTracker(frame_w=1920, frame_h=1080)

# Each detection: [x1, y1, x2, y2, confidence]
detections = [
    [120, 80, 170, 130, 0.91],
    [300, 200, 350, 250, 0.87],
]

# Call once per frame
output = tracker.update(detections)
# output: np.array of shape (N, 20) — one row per confirmed track
```

For multi-session use, reset the ID counter between sessions:

```python
from qsort_tracker import reset_id_counter
reset_id_counter(start=1)
```

---

## Fine-Tuning Parameters

Start with defaults. If tracking quality is poor for your data, adjust these in order:

### Detection Gate

| Parameter | Default | Effect |
|-----------|---------|--------|
| `MIN_CONF` | 0.25 | Minimum detector confidence accepted |

### Distance Gate

| Parameter | Default | Effect |
|-----------|---------|--------|
| `BOLTZ_DIST_FRAC` | 0.16 | Search radius as fraction of frame diagonal (~25% of 1080p diagonal) |

### Track Lifecycle

| Parameter | Default | Effect |
|-----------|---------|--------|
| `MAX_MISSED` | 50 | Frames before track is deleted |
| `BIRTH_CONFIRM` | 2 | Matches before track is exported |
| `MIN_AGE_DELETE` | 4 | Never delete tracks younger than this |

### Reliability (R)

| Parameter | Default | Effect |
|-----------|---------|--------|
| `R_INIT` | 0.80 | Starting reliability |
| `R_BOOST` | 0.04 | Per matched frame increase |
| `R_DECAY` | 0.03 | Per missed frame decrease |
| `R_FLOOR_ACTIVE` | 0.12 | Floor below which unconfirmed tracks die |

### Association Cost Weights (must sum considerations)

| Parameter | Default | Term |
|-----------|---------|------|
| `COST_DIST_W` | 0.35 | Euclidean distance |
| `COST_BOLTZ_W` | 0.25 | Boltzmann energy |
| `COST_DIR_W` | 0.20 | Direction alignment |
| `COST_TENSOR_W` | 0.10 | Tensor mismatch |
| `COST_IOU_W` | 0.10 | Inverse IoU |

All terms are normalised to `[0, 1]` before weighting, so the weights are directly comparable.

### Speed Thresholds (L5)

| Parameter | Default | Notes |
|-----------|---------|-------|
| `SPEED_S1` | 2.0 px/frame | Stopped → Gliding |
| `SPEED_S2` | 15.0 px/frame | Gliding → Burst |

Adjust these for your camera resolution and subject speed.

---

## API Reference

### `QSortPhysicsTracker(frame_w, frame_h)`

| Argument | Type | Description |
|----------|------|-------------|
| `frame_w` | int | Frame width in pixels |
| `frame_h` | int | Frame height in pixels |

### `.update(dets) → np.ndarray`

| Argument | Type | Description |
|----------|------|-------------|
| `dets` | list of `[x1, y1, x2, y2, conf]` | Raw detections for this frame |

Returns `np.ndarray` of shape `(N, 20)`. Empty array `(0, 20)` if no confirmed tracks.

### `reset_id_counter(start=1)`

Resets the global track ID counter. Call between independent sessions.

---

## Output Format

Each row in the output array contains 20 values:

| Index | Field | Description |
|-------|-------|-------------|
| 0–3 | `x1, y1, x2, y2` | Bounding box |
| 4 | `id` | Track ID (integer) |
| 5–6 | `vx, vy` | Velocity (px/frame) |
| 7–8 | `ax, ay` | Acceleration (px/frame²) |
| 9–10 | `jx, jy` | Jerk (px/frame³) |
| 11 | `kappa` | Curvature |
| 12–13 | `px, py` | Effective momentum |
| 14 | `sigma_p` | Positional uncertainty |
| 15 | `sigma_theta` | Angular uncertainty |
| 16 | `R` | Reliability [0, 1] |
| 17–19 | `theta_A, theta_B, theta_C` | Bloch sphere angles |

---

## Ablation Testing

Test all 32 layer combinations (on/off for L2–L6) across your scenarios:

```python
from qsort_tracker import run_ablation

scenarios = {
    "open_water": list_of_det_lists_scenario_1,
    "crowded":    list_of_det_lists_scenario_2,
}

results = run_ablation(scenarios, frame_w=1920, frame_h=1080)
```

Prints a comparison table of unique track births per scenario per configuration. Useful for identifying which layers contribute to your specific dataset.

---

## Changelog from Previous Version

| Tag | Fix |
|-----|-----|
| `[C1]` | Confirmed track status is permanent — no oscillation |
| `[C2]` | Double R modification eliminated — single unified R system |
| `[C3]` | All cost terms normalised to [0,1] — weights now meaningful |
| `[H1]` | Polynomial fits real detection history only (separate `real_hist`) |
| `[H2]` | Bloch speed thresholds corrected for real video scale |
| `[H3]` | Distance gate derived from frame diagonal, not hardcoded px |
| `[H4]` | `BlochEngine` + `TensorEngine` are module-level singletons |
| `[H5]` | Debug logging uses Python `logging`, not file I/O |
| `[H6]` | Python `or` bug fixed in `_poly_predict_next` |
| `[M1]` | Re-ID uses predicted `mu_x` not stale `x` |
| `[M2]` | `bbox` always stored as numpy array |
| `[M3]` | `qs_safe_norm` alias removed |
| `[M4]` | ID counter is thread-safe with a lock |
| `[M5]` | Curvature smoothed over 3-frame velocity window |
| `[M6]` | `MIN_CONF` filters low-confidence detections before cost matrix |
| `[L1]` | Tensor alpha values exposed as tunable parameters |
| `[L2]` | Frame size passed to tracker init for resolution-aware thresholds |
| `[L3]` | Ablation runner included as `run_ablation()` |

---

## Known Limitations

- **Untested on diverse datasets** — validated on fish tracking video; may need re-tuning of speed thresholds, distance fractions, and cost weights for other domains (pedestrians, vehicles, drones, etc.).
- **Single-class** — no class-conditioned association; detections from different object classes are treated identically.
- **No appearance model** — purely motion-based; re-ID under long occlusion relies on kinematics alone.
- **Fixed frame rate** — `DT = 1.0` assumes constant frame interval; variable-FPS video will need `DT` to reflect actual elapsed time.
- **2D only** — all physics operate in image-plane coordinates.

---

## Contributing

If you want to extend this tracker, useful directions include:

- Appearance embedding integration (ReID features alongside the cost matrix)
- Variable `DT` support for non-constant frame rates
- 3D extension using stereo or depth-camera input
- Benchmarking on MOTChallenge or Fish4Knowledge datasets
- Domain-specific parameter sweeps (drones, vehicles, sports)

Basic familiarity with classical mechanics (velocity, curvature) and quantum mechanics notation (Bloch sphere, tensor products) is helpful but the code itself is pure NumPy/SciPy.

---

## License

Contact author for further information, however it has MIT license and free for any kind of use without any warrenty.
