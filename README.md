# 🔋 Battery-Aware A* Rover Pathfinding Simulator

### 📌 Project:
**Analysis of A\* Heuristics for Pathfinding in a Battery-Constrained Rover on a Dynamic Square Grid**

---

## 📖 Overview

This project implements an interactive **A\***-based rover simulation to evaluate how different heuristic strategies perform in a **dynamic**, **hazard-filled** grid world with **limited battery capacity**.

The rover must reach its goal while:
- Managing **power consumption**
- Avoiding **obstacles and cliffs**
- Adapting to **unexpected terrain changes**
- Utilizing **recharge stations** effectively
- **Replanning** in real time when emergencies occur

This simulator offers visualization and analytical comparison making it suitable for **research** and **academic evaluation** of A\* heuristic performance.

---

## 🧠 Supported Heuristics

| Heuristic | Description |
|----------|-------------|
| Manhattan | Standard grid-based shortest distance |
| Euclidean | Straight-line heuristic |
| Terrain Penalized (Magnetic Field) | Uses artificial potential field: repulsive cost for dangerous terrain |
| Path of Least Regret | Penalizes high-variance terrain to reduce risk-taking |

Each heuristic uses a **distinct visual color** during path rendering.

---

## 🗺 Terrain Types

| Terrain | Cost | Color | Behavior |
|--------|------|-------|---------|
| flat | 5 | Green | Default terrain |
| sandy | 10 | Yellow | Higher energy consumption |
| rock | 1000 | Brown | Impassable |
| recharge | 5 | Blue | Battery fully refilled |
| hazardous | 15 | Orange/Red | Avoid when possible → triggers rerouting |
| trap | 20 | Purple | Can lead to forced replanning / block off paths |
| cliff | 10000 | Dark gray | Deadly / fully blocked |

---

## 🔋 Battery-Aware Navigation Features

✔ Live battery indicator  
✔ Auto-routing through recharge points  
✔ Emergency reroute when battery gets too low  
✔ Rover may **fail** if unable to replan after hazards  
✔ Proper cost-based battery consumption

---

## 🌪 Dynamic Environment Adaptation

Real-time random terrain changes simulate a volatile environment:
- Terrain on the rover's planned path may suddenly change
- Rover **detects**, **backtracks**, and **replans**
- Past path remains visible for analysis

This models real-world challenges in Mars / Lunar rover missions.

---

## 🖥 Graphical User Interface

Features include:
- Click-to-set **Start** and **Goal**
- View & compare results of **all heuristics**
- Radar showing **5×5 surrounding grid**
- Export & Import grid as `.json`
- Adjustable animation speeds
- Path length & cost reports

---

## ▶ Installation & Execution

### Requirements
- Python **3.8+**
- No external dependencies — uses only Python standard libraries

## Python Code File 
-
### Run the Simulator
```bash
python Rover_Sim_2.py
