# Rokey-Collaboration1

# 🦾 Main Domino Project

> A robotics project that stacks dominoes in a staircase shape and knocks them down using a **Doosan M0609** industrial robot arm.  
> Built with **ROS 2 (Humble)** and controlled via Python using **precise motion planning, force control**, and **path compensation**.

---

## 🎥 Demo Video

👉 [Watch the demo](https://your-demo-link.com)  
_A robot arm picks and places dominoes in a staircase formation, then knocks them over._

---

## 🧠 Overview

### Objective
- Stack dominoes automatically in a precise stair-step formation.
- Demonstrate realistic pick-and-place control using Doosan’s Python API.
- Integrate path planning with pose correction and motion execution.

### Technologies Used
- ROS 2 (Humble)
- Doosan Robot API (M0609)
- Python 3, NumPy
- tf_transformations
- Skeleton extraction & sampling
- Force-based movement control

---

## ⚙️ System Flow

```
Input Image
    ↓
Binarization & Skeleton Extraction
    ↓
Path Sampling & Stair Logic
    ↓
Pose Transformation (XYZ + Yaw)
    ↓
Doosan Robot Motion Execution
```

---

## 📁 Project Structure

```
domino_project/
├── domino_move/                 # Robot motion execution
│   ├── real_move.py
│   ├── stair_pose_utils.py
├── image_preprocessor/         # Image → Skeleton processing
│   └── preprocessor.py
├── tf_utils/                   # Quaternion and yaw utilities
│   └── transform_utils.py
├── data/
│   └── domino_image.png        # Input image
└── README.md
```

---

## 🚀 How to Run

### 1. Source ROS 2 and your workspace

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash
```

### 2. Launch the main motion node

```bash
ros2 run domino_move real_move
```

This will command the robot to execute the full domino stacking and knocking sequence.

---

## 🔧 Key Components

| Function | Description |
|---------|-------------|
| `update_all_converted_poses_with_yaw_based_offset()` | Adjusts all poses using yaw-based offset logic |
| `apply_stair_pose_from_stack_logic()` | Adds Z-offset for stair-step stacking |
| `pick()` / `place()` | Executes pick-up and placement motions |
| `movel()` | Executes straight-line Cartesian motion |

---

## ✅ Highlights

- Stair-shaped stacking: center domino raised by 40mm, sides by 20mm
- Auto Z compensation based on index and total height
- Final domino is knocked over using speed or force control
- Real-time console logging for action feedback

---

## 🌱 Future Improvements

- Add YOLO-based domino detection and pose estimation
- Integrate optimal path planning and reordering
- Provide Gazebo simulation support

---

## 👤 Author

- **Junmo Han (한준모)**
- GitHub: [weedmo](https://github.com/weedmo)

## ⚙️ Doosan Robot move gear

This project demonstrates a simple **Pick & Place** system using the **Doosan M0609** collaborative robot.

The robot picks up 4 gears from predefined positions and places them at corresponding target positions using optional **force control** for accurate placement.

[![Demo Video](https://img.youtube.com/vi/bPqb-HWBBT0/0.jpg)](https://youtu.be/bPqb-HWBBT0)  
▶️ [Watch on YouTube](https://youtu.be/bPqb-HWBBT0)

### demo Run

```bash
ros2 run assignment move_gear
```


This project demonstrates a **Pick & Place system** using the **Doosan M0609 collaborative robot**.  
The robot detects the Z-contact point of each block using **force sensing**, and then sorts the blocks into columns based on height.

- **Input**: 3x3 block grid  
- **Output**: Sorted into 3 columns based on contact height (`z`)


https://github.com/user-attachments/assets/b0cc4195-48fa-41e2-95e9-36330a62b2d2

### demo run
```bash
ros2 run assignment move_block
```
## 🧱 Doosan Robot LEGO
This project demonstrates a Pick & Place system using the Doosan M0609 collaborative robot.
The robot picks up LEGO blocks using force sensing to detect contact, then places them precisely onto a target board to rebuild a structure.

Input: Randomly placed LEGO blocks on a base plate

Output: Rearranged and attached to a new location based on predefined layout



https://github.com/user-attachments/assets/869d0c86-701b-425b-8b63-eac0a6d21d30

### demo run
```bash
ros2 run assignment move_lego
```
---

## 🦾 Doosan Robot move object

[Screencast from 06-18-2025 12_43_21 PM.webm](https://github.com/user-attachments/assets/d920f07d-990f-41e8-83e0-f0d15d077895)

### demo run
```bash
ros2 run assignment move_object
```
---
## 👥 Contributors

Thanks to these wonderful people who have contributed to this project:

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/weedmo">
        <img src="https://github.com/weedmo.png" width="100px;" alt="weedmo"/><br />
        <sub><b>weedmo</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/jsbae-RL">
        <img src="https://github.com/weedmo.png" width="100px;" alt="jsbae-RL"/><br />
        <sub><b>jsbae-RL</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/DONGHO1206">
        <img src="https://github.com/DONGHO1206.png" width="100px;" alt="DONGHO1206"/><br />
        <sub><b>DONGHO1206</b></sub>
      </a>
    </td>


</table>


---

