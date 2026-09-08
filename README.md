# 🐦 OpenBird-artnfull (Bipedal VTOL Robot Bird)

> **Physical AI Companion Robot Simulation Platform by artnfull**  
> *True Digitigrade Bipedal Locomotion + 6-DOF VTOL Flight Simulation (MuJoCo & Python)*

---

## ✨ Key Features

1. **🦿 True Digitigrade Bipedal Locomotion**
   - Biologically inspired avian legs: Hip $\rightarrow$ Reversed Knee $\rightarrow$ Shin $\rightarrow$ Ankle $\rightarrow$ Round Soles.
   - Forward lean bias stabilization and dynamic walking (Forward, Backward, Turn).
2. **🪽 Vertical Stowed & Deployed Duct Wings**
   - Folded vertically behind the back during walking, deployed horizontally (180°) for VTOL flight.
   - Smooth 4-stage flight FSM: Stand $\rightarrow$ Spool $\rightarrow$ Climb $\rightarrow$ Altitude Hold & Hover.
3. **🦅 Look-Down Eye-Gaze & Beak Pitch**
   - Independent eye-gaze and beak pitch control (`neck_pitch`) to look down 55° for ground observation while keeping wings perfectly horizontal.
4. **📷 Stereo Eye Cameras & FPV View**
   - Dual eye cameras (`left_eye_cam`, `right_eye_cam`) and center FPV camera (`head_fpv_cam`).
5. **🧠 Autonomous Simulation Brain**
   - Built-in demonstration routines for curiosity roaming, pecking, hopping, and self-righting knockdown recovery.

---

## 🎮 Control Guide

| Key | Action | Description |
| :---: | :---: | :--- |
| `A` / `D` | **Yaw Turn** | Turn left / right (Walking turn / Flight yaw) |
| `↑` / `↓` | **Forward / Backward** | Walk forward/backward or Fly forward/backward |
| `←` / `→` | **Strafe** | Sideways translation in air |
| `R` / `F` | **Altitude** | Altitude Climb (+15cm) / Descend (-15cm) |
| `SPACE` | **Stop / Hover** | Stand still (Ground) / Precision Hover (Air) |
| `1` $\rightarrow$ `2` $\rightarrow$ `3` | **Flight Sequence** | Deploy Wings $\rightarrow$ Takeoff / Hover $\rightarrow$ Land & Fold |
| `C` | **Camera Switch** | 3rd-person Chase $\rightarrow$ Head FPV $\rightarrow$ Left Eye $\rightarrow$ Right Eye |
| `V` / `B` | **Eye-Gaze Pitch** | Look down 55° (V) / Look forward (B) |
| `G` / `Y` / `K` / `P` | **Gestures** | Peck (`G`), Crouch (`Y`), Knockdown (`K`), Auto-Right (`P`) |
| `Q` / `ESC` | **Exit** | Close simulator |

---

## 🚀 Quick Start

Double-click the batch files to launch instantly:

* **`run_ai_auto.bat`** : Autonomous AI Brain exploration mode
* **`run_manual.bat`** : 6-DOF Manual keyboard flight & walk control mode

---

## 📁 Directory Structure

```text
1_github_public/
├── robot_bird/
│   ├── models/
│   │   └── robot_bird.xml       # MuJoCo 3D physics model
│   ├── controller.py           # 6-DOF flight & bipedal walking kinematics FSM
│   ├── ai_brain.py             # Autonomous behavior demonstration engine
│   └── recorder.py             # Video screen capture utility
├── run_ai_robot_bird.py        # Autonomous AI simulator launcher
├── run_robot_bird.py           # Interactive manual flight launcher
├── run_ai_auto.bat             # One-click AI launcher
├── run_manual.bat              # One-click Manual launcher
├── LICENSE                     # Software Apache-2.0 License
├── LICENSE_DESIGN              # Hardware CC BY-NC-SA 4.0 License
└── README.md                   # Project overview & documentation
```

---

## 📜 Licenses
- **Software Code:** [Apache License 2.0](LICENSE)
- **3D Models & Mechanical Designs:** [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 (CC BY-NC-SA 4.0)](LICENSE_DESIGN) (Commercial reproduction prohibited)

---

## ⚠️ Support Policy & Disclaimer

- **Standalone Research Preview:** This project is provided on an **"AS-IS" basis** as a demonstration for the Physical AI & Robotics community.
- **No Individual Technical Support:** The maintainers **do not provide 1-on-1 customer service, individual hardware consulting, or personal troubleshooting**.
- **Issue Tracker Guidelines:** The issue tracker is strictly reserved for critical bug reporting. Questions regarding basic Python setup or personal modifications will be closed without individual response.

