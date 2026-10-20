# 16-782: Planning and Decision-Making in Robotics
## Homework 2: Planning for High-DoF Arm

This repository contains the implementation of sampling-based motion planning algorithms for high-degree-of-freedom planar manipulators in $S^1 \times \cdots \times S^1$ toroidal configuration space.

---

### Planners Implemented

| ID | Planner | Description |
|:---:|:---|:---|
| `0` | **Vanilla RRT** | Single-tree Rapidly-exploring Random Tree with angular wrapping and tunable goal bias. |
| `1` | **RRT-Connect** | Dual-tree bidirectional RRT with greedy extension step until connection. |
| `2` | **Vanilla RRT\*** | Asymptotically optimal RRT with metric-scaled near-neighbor rewiring. |
| `3` | **PRM** | Probabilistic Roadmap with $k$-nearest neighbors and roadmap densification. |
| `4` | **RRT\* + Node Rejection (NR)** | Extra Credit: Rejects samples exceeding current best cost $c_{\text{best}}$. |
| `5` | **RRT\* + Local Biasing (LB)** | Extra Credit: Concentrates samples locally along the discovered solution path. |
| `6` | **RRT\* + Combined (CO)** | Extra Credit: Combines Local Biasing with Node Rejection pruning. |

---

### Compilation Instructions

#### Option 1: CMake (Recommended)
```bash
cd code
mkdir -p build
cd build
cmake ..
make -j4
```
This produces `planner`, `verifier`, and `generate_pairs` executables inside `code/build/`.

#### Option 2: Direct g++
```bash
cd code/build
g++ -O3 -std=c++14 ../src/planner.cpp -o planner
g++ -O3 -std=c++14 ../src/verifier.cpp -o verifier
```

---

### Running the Planner

Execute the planner from within the `code/build/` directory:
```bash
./planner [mapFile] [numofDOFs] [startAnglesCommaSeparated] [goalAnglesCommaSeparated] [whichPlanner] [outputFile]
```

**Example:**
```bash
./planner map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 1 myOutput.txt
```

---

### Automated Grading & Verification

Run the official test suite from `code/scripts/`:
```bash
cd code/scripts
python3 grader.py
```
This tests all core planners against the official maps and verifies collision-free trajectories and endpoint consistency via `verifier`.

---

### Visualizing Solutions

To generate an animated GIF of a planned motion:
```bash
cd code/scripts
python3 visualizer.py ../output/myOutput.txt --gifFilepath ../output/myMotion.gif
```

---

### Directory Structure

```
.
├── 16782_fall26_HW2.pdf     # Homework specification document
├── README.md                # Project documentation and compilation guide
├── .gitignore               # Clean git tracking exclusions
└── code/
    ├── CMakeLists.txt       # Build system definition
    ├── build/               # Contains precompiled planner and verifier binaries
    ├── maps/                # Map benchmark definitions (map1..map5, map_challenge)
    ├── output/              # Benchmarks, Overleaf report tables, and visualizations
    ├── scripts/             # Grading, visualizer, and experiment scripts
    └── src/
        ├── planner.cpp      # Core planning implementations (RRT, RRT-Connect, RRT*, PRM, Heuristics)
        ├── verifier.cpp     # Path collision and boundary validation tool
        └── generate_pairs.cpp # Collision-free problem instance generator
```
