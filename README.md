# Quantum Sort Tracker
# Quantum-Inspired Object Tracking & Counting System (QSort-A)

**Author:** Deepak Pandey  
**Independent Researcher — Quantum Motion & Tracking Systems**

---

# 1. Introduction

QSort-A is a next-generation, quantum-inspired high-speed **object tracking and counting system** designed for real-time video (60–120 FPS).  
It solves failure modes found in traditional SORT/Kalman filters, especially when objects:

- deform
- flip orientation
- merge and split
- occlude frequently
- collapse temporarily
- move non-linearly
- vary in appearance or shape
- exist in dense environments

QSort-A models multi-order motion behaviour (velocity, acceleration, jerk, curvature) instead of relying solely on IoU or linear motion.  
This design enables accurate, human-like tracking in unpredictable real-world environments.

---

# 2. Conceptual Foundation

Traditional trackers assume rigid, smooth motion.  
Real object motion is:

- elastic  
- noisy  
- deforming  
- chaotic  
- interacting  
- biological or natural  

QSort-A is based on these principles:

### Motion carries identity  
Identity is reconstructed from movement patterns, not appearance.

### Quantum mechanics inspiration  
State transitions depend on a mixture of position, momentum, and probabilistic evolution.

### Statistical mechanics  
Motion is not deterministic—small fluctuations define behaviour.

### Biological motion  
Real movement includes bending, turning, deforming, and flipping.

QSort-A uses these ideas to track objects even in the presence of deformation, occlusion, or collapse.

---

# 3. Mathematical Framework

Bounding box format:
[x1, y1, x2, y2, confidence]


Centre point:

cx = (x1 + x2) / 2
cy = (y1 + y2) / 2


Motion history for a track:
P = [(cx1, cy1), (cx2, cy2), ..., (cxn, cyn)]


From history, QSort-A calculates:

### Velocity


v = Pn – P(n-1)
|v| = speed


### Acceleration
a = v(n) – v(n-1)

### Jerk
j = a(n) – a(n-1)

### Curvature κ
Approximate curvature from velocity and acceleration:

κ ≈ |v × a| / |v|^3

These features form the motion signature:
M = [vx, vy, |v|, |a|, |j|, κ]


This signature is compared with new detections for identity continuity.

---

# 4. Cost Function and Matching

QSort-A uses a hybrid cost model with the Hungarian algorithm.  
Components include:

1. **Distance cost**  
2. **Velocity-match penalty**  
3. **Jerk penalty (behaviour flips)**  
4. **Curvature penalty**  
5. **IoU as a minor rejection constraint**  

This prevents identity switches even during deformation or occlusion.

---

# 5. Identity Preservation

QSort-A includes three identity-preserving mechanisms.

### 5.1 Collapse-Memory  
When a track disappears briefly, its last location is saved.  
If a new detection appears nearby within a small frame window, the old ID is restored.

### 5.2 Freeze Window  
After collapse, QSort-A freezes ID creation for a few frames around the same region.  
This prevents “ID explosions” during shape collapse.

### 5.3 Behaviour Matching  
Identity is maintained if velocity, jerk, and curvature patterns match historical motion.

These mechanisms allow stable long-term tracking of non-rigid objects.

---

# 6. Object Counting System

Counting uses a **line-crossing model with tolerance bands**.

- Count each ID only once  
- Flip or shape deformation does not double-count  
- Near-line motion without crossing is ignored  
- Re-identification does not alter counts  

This ensures accurate counting in high-speed, high-density situations.

---

# 7. Software Architecture

Three-thread asynchronous architecture:

### Capture Thread
Continuous frame capture at camera FPS.

### Processing Thread
YOLO detection → QSort-A tracking → counting → overlay → video saving.

### Display Thread
Real-time GUI without blocking processing.

Enables stable 60–120 FPS operation depending on hardware/GPU.

---

# 8. File Structure
QSort-A-Object-Tracker/

├── Mainfishcount_A_best.py # Main program
├── qsort_tracker_A.py # QSort-A tracker implementation
├── last1.engine # YOLO engine file
├── ll.mp4 # Example footage
├── results/ # Raw and processed video outputs
└── docs/ # Theory, diagrams, notes


---

# 9. Usage

Run the system:


Features shown:

- Real-time object tracking  
- Object ID overlays  
- Velocity, curvature, jerk visualisation  
- Counting line  
- Total object count  
- Auto-saving raw + processed video (90 FPS)  

---

# 10. Application Domains

QSort-A is general-purpose.

### Biological Tracking
- animals  
- insects  
- microscopic cells  
- protein motion  
- particle movement  

### Industrial
- robotics  
- conveyor tracking  
- automation QA  
- machine vision  

### Surveillance
- pedestrians  
- vehicles  
- drones  
- multi-object scenes  

### Scientific Research
- fluid dynamics  
- Brownian motion  
- multi-body interactions  
- motion analysis  

---

# 11. Comparison with SORT/Kalman

| SORT Limitations | QSort-A Advantages |
|------------------|--------------------|
| ID switches during flips | Behaviour-based jerk + curvature matching |
| Overcounts when collapsing | Collapse-memory + freeze windows |
| High reliance on IoU | Motion signature dominates |
| New ID creation too easily | Frozen-ID spawning + behaviour restoration |
| Fails on non-rigid motion | Multi-order physics-based modelling |
| Unstable in dense scenes | Hybrid cost + motion constraints |

QSort-A is robust in conditions where SORT fails.

---

# 12. Roadmap

- QSort-B (trajectory-cluster matching)  
- QSort-C (behaviour-signature dominant tracker)  
- Multi-camera fusion  
- RL-based long-term ID consistency  
- Automatic dataset generation  
- Embedded device optimisation (Jetson, Raspberry Pi 5)  
- Web dashboard integration  
- Behaviour classification modules  

---

# 13. Citation

If you use QSort-A in research:
Deepak Pandey, "Quantum-Inspired Multi-Motion Object Tracker (QSort-A)", 2025.


---

# 14. License

MIT License  


---

# 15. Acknowledgements

Developed through real-world high-speed motion tracking challenges.  
Special thanks to colleagues, mentors, and family for continuous support. Special 
thanks to R&D crew ( Neville Doyle, Bradly Jones, Jason Clark) of Humpty Doo Barramundi. All this 
idea came after doing R&D fish counting project. 




