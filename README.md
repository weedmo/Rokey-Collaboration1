# Rokey-Collaboration1
## ⚙️ Doosan Robot move gear

## 🦾 Doosan Robot move block

This project demonstrates a **Pick & Place system** using the **Doosan M0609 collaborative robot**.  
The robot detects the Z-contact point of each block using **force sensing**, and then sorts the blocks into columns based on height.

- **Input**: 3x3 block grid  
- **Output**: Sorted into 3 columns based on contact height (`z`)


https://github.com/user-attachments/assets/b0cc4195-48fa-41e2-95e9-36330a62b2d2

### demo run
```bash
ros2 run assignment move_block
```
## 🧱🤖 Doosan Robot LEGO
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

