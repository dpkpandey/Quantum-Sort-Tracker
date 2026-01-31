# QSort-A: Quantum-Inspired Object Tracking & Counting System

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

**Author:** Deepak Pandey  
**Organization:** Quantum Motion & Tracking Systems

---

## Overview

QSort-A is a next-generation, quantum-inspired object tracking and counting system designed for high-speed video processing (60–120 FPS). It addresses critical failure modes in traditional SORT/Kalman filter-based trackers by modeling multi-order motion dynamics rather than relying solely on Intersection over Union (IoU) or linear motion assumptions.

### Key Advantages

- **Robust Identity Preservation:** Maintains consistent IDs through deformation, occlusion, and temporary disappearance
- **Multi-Order Motion Modeling:** Tracks velocity, acceleration, jerk, and curvature for accurate predictions
- **Real-Time Performance:** Achieves 60–120 FPS on modern hardware with GPU acceleration
- **Accurate Counting:** Line-crossing detection with tolerance bands prevents double-counting
- **Non-Rigid Object Support:** Handles flexible, deforming, and shape-changing objects

---

## Problem Statement

Traditional tracking systems fail when objects exhibit:

- Elastic deformation and shape changes
- Orientation flips and rotations
- Temporary merging and splitting
- Frequent occlusions
- Non-linear motion patterns
- Appearance variations
- High-density environments with multiple interactions

QSort-A solves these challenges through quantum-inspired probabilistic state transitions and physics-based motion modeling.

---

## Conceptual Foundation

### Core Principles

**Motion Carries Identity**  
Object identity is reconstructed from movement patterns rather than visual appearance alone.

**Quantum Mechanics Inspiration**  
State transitions depend on a probabilistic mixture of position, momentum, and temporal evolution.

**Statistical Mechanics**  
Motion is inherently stochastic—small fluctuations define long-term behavior.

**Biological Motion Understanding**  
Real-world movement includes bending, turning, deforming, and flipping that must be accommodated.

---

## Mathematical Framework

### Bounding Box Representation

```
Format: [x1, y1, x2, y2, confidence]
Center: (cx, cy) = ((x1 + x2) / 2, (y1 + y2) / 2)
```

### Motion History

For each tracked object, we maintain a position history:

```
P = [(cx₁, cy₁), (cx₂, cy₂), ..., (cxₙ, cyₙ)]
```

### Motion Derivatives

**Velocity:**
```
v = Pₙ - Pₙ₋₁
|v| = speed
```

**Acceleration:**
```
a = vₙ - vₙ₋₁
```

**Jerk (rate of acceleration change):**
```
j = aₙ - aₙ₋₁
```

**Curvature:**
```
κ ≈ |v × a| / |v|³
```

### Motion Signature

These features combine to form a unique motion signature for each track:

```
M = [vₓ, vᵧ, |v|, |a|, |j|, κ]
```

This signature is compared with new detections to maintain identity continuity.

---

## Cost Function & Matching

QSort-A employs a hybrid cost model optimized via the Hungarian algorithm:

1. **Spatial Distance:** Euclidean distance between predicted and detected positions
2. **Velocity Matching:** Penalizes deviations from expected velocity vectors
3. **Jerk Penalty:** Identifies sudden behavioral changes (e.g., direction reversals)
4. **Curvature Penalty:** Accounts for turning and curved trajectories
5. **IoU Constraint:** Used as a weak rejection filter rather than primary metric

This multi-factor approach prevents identity switches during complex motion scenarios.

---

## Identity Preservation Mechanisms

### 1. Collapse-Memory System

When a track temporarily disappears:
- Last known position and motion signature are cached
- If a new detection appears nearby within N frames, the cached ID is restored
- Prevents unnecessary ID creation for brief occlusions

### 2. Freeze Window

After track collapse:
- ID creation is frozen in the spatial vicinity for several frames
- Prevents "ID explosion" when objects split and rejoin
- Maintains tracking continuity through shape deformation

### 3. Behavioral Matching

Identity is preserved when:
- Velocity patterns align with historical data
- Jerk signatures indicate consistent behavioral patterns
- Curvature matches expected trajectory evolution

These mechanisms enable stable long-term tracking of non-rigid, deformable objects.

---

## Counting System

### Line-Crossing Detection

The counting system uses a **virtual line with tolerance bands**:

- Each ID is counted only once per crossing
- Shape deformation and orientation flips do not trigger multiple counts
- Near-line motion without crossing is correctly ignored
- Re-identification after occlusion does not alter count totals

This ensures accurate enumeration in high-speed, high-density scenarios.

---

## System Architecture

### Three-Thread Asynchronous Design

**Capture Thread**
- Continuous frame acquisition at native camera FPS
- Minimizes frame drops and timing jitter

**Processing Thread**
- YOLO object detection
- QSort-A tracking and ID assignment
- Count computation
- Video overlay rendering
- Output video encoding

**Display Thread**
- Real-time GUI updates
- Non-blocking visualization
- Performance metrics display

This architecture enables stable 60–120 FPS operation depending on hardware configuration.

---

## Project Structure

```
QSort-A-Object-Tracker/
├── Mainfishcount_A_best.py    # Main application entry point, this is your counting program, you can have your own. 
├── qsort_tracker_A.py         # Core QSort-A tracking implementation, this is the tracker you have it now here. This is small one. In future I will update with main tracker which works for 17 vectors.
├── last1.engine               # YOLO TensorRT engine file, you can use any pytorch model, this is your trained model
├── ll.mp4                     # Example test footage, this is footage, but instead of this you can use your camera feed as well
├── results/                   # Output directory for processed videos
│   ├── raw/                   # Raw camera footage
│   └── processed/             # Annotated tracking videos
└── docs/                      # Documentation and diagrams
    ├── theory.md              # This is the main concept behind this tracker, I will upload pdf file for this as well. 
    ├── architecture.md
    └── diagrams/
```

---

## Installation

### Requirements

- Python 3.8 or higher
- CUDA-compatible GPU (recommended)
- TensorRT (for optimized inference)

### Dependencies

```bash
pip install numpy opencv-python scipy
pip install torch torchvision  # PyTorch for YOLO
pip install tensorrt           # For inference optimization
```

### Setup

```bash
git clone https://github.com/yourusername/QSort-A-Object-Tracker.git
cd QSort-A-Object-Tracker
pip install -r requirements.txt
```

---

## Usage

### Basic Execution

```bash
python Mainfishcount_A_best.py --input ll.mp4 --output results/
```

### Command-Line Options

```bash
python Mainfishcount_A_best.py \
    --input <video_path> \
    --output <output_directory> \
    --fps <target_fps> \
    --engine <yolo_engine_path> \
    --display  # Show real-time visualization
```

### Real-Time Features

The system displays:
- Object bounding boxes with unique IDs
- Velocity vectors
- Curvature indicators
- Jerk visualization (behavioral changes)
- Counting line overlay
- Total object count
- Processing FPS

### Output

The system automatically saves:
- Raw video stream (90 FPS)
- Processed video with annotations
- Tracking statistics and count logs

---

## Applications

### Biological Research
- Animal behavior tracking
- Insect swarm dynamics
- Microscopic cell movement
- Protein motion analysis
- Particle tracking in fluid systems

### Industrial Automation
- Robotic vision systems
- Conveyor belt monitoring
- Manufacturing quality assurance
- Automated inspection systems

### Surveillance & Security
- Pedestrian flow analysis
- Vehicle traffic monitoring
- Drone tracking
- Crowd density estimation

### Scientific Research
- Fluid dynamics visualization
- Brownian motion studies
- Multi-body interaction analysis
- High-speed motion capture

---

## Performance Comparison

| Feature | Traditional SORT | QSort-A |
|---------|-----------------|---------|
| ID switching during flips | Frequent | Rare (jerk + curvature matching) |
| Overcounting on collapse | Common | Prevented (collapse-memory) |
| Motion model | Linear (velocity only) | Multi-order (velocity, acceleration, jerk) |
| IoU dependency | High | Low (motion-signature dominant) |
| Non-rigid object support | Poor | Excellent |
| Dense scene performance | Unstable | Robust |
| ID creation strategy | Immediate | Controlled (freeze windows) |

---

## Roadmap

### Planned Features

- **QSort-B:** Trajectory-cluster matching for long-term tracking
- **QSort-C:** Behavior-signature dominant tracker with learned patterns
- **Multi-Camera Fusion:** Synchronized tracking across camera networks
- **Reinforcement Learning:** Adaptive ID consistency optimization
- **Auto-Dataset Generation:** Synthetic training data creation
- **Edge Deployment:** Optimization for Jetson, Raspberry Pi 5
- **Web Dashboard:** Browser-based monitoring and configuration
- **Behavior Classification:** Automated activity recognition modules

---

## Citation

If you use QSort-A in your research, please cite:

```bibtex
@software{pandey2025qsort,
  author = {Pandey, Deepak},
  title = {QSort-A: Quantum-Inspired Multi-Motion Object Tracker},
  year = {2025},
  publisher = {GitHub},
  organization = {Quantum Motion & Tracking Systems}
}
```

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Acknowledgements

This research was developed through real-world high-speed motion tracking challenges in aquaculture environments. 

**Special Thanks:**
- R&D Team at Humpty Doo Barramundi: Neville Doyle, Bradley Jones, Jason Clark
- Family and colleagues for continuous support
- The computer vision research community

**Project Origin:**  
The core concepts emerged from fish counting R&D challenges requiring robust tracking of highly deformable objects in complex aquatic environments.

---

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss proposed modifications.

### Development Setup

```bash
git clone https://github.com/yourusername/QSort-A-Object-Tracker.git
cd QSort-A-Object-Tracker
pip install -r requirements-dev.txt
```

---

## Contact

**Deepak Pandey**  
Quantum Motion & Tracking Systems

For questions, suggestions, or collaboration inquiries, please open an issue on GitHub.

---

## Frequently Asked Questions

**Q: What hardware is recommended?**  
A: NVIDIA GPU with CUDA support (RTX 3060 or better), 8GB+ RAM, modern multi-core CPU, it depends what you want to do, for complex system use better GPUs

**Q: Can this track transparent or semi-transparent objects?**  
A: Yes, as long as the detector (YOLO) can identify them. QSort-A handles the tracking. This is first step of tracker only but for whole calculation or detection follow https://q-ots-website.pages.dev/ 

**Q: What FPS can I expect?**  
A: 60-120 FPS on modern hardware with GPU acceleration. CPU-only: 15-30 FPS.

**Q: Does it work with custom YOLO models?**  
A: Yes, any YOLO detector producing [x1, y1, x2, y2, confidence] format is compatible, you can make compatible with any existing models becuase this tracker works upto 17 verctors co-ordinates.

**Q: How does it compare to DeepSORT?**  
A: QSort-A uses motion physics instead of appearance features, making it more robust to visual changes and deformations.

---

*Last Updated: January 2025*

