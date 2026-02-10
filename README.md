# DFL-QP Unicycle Controller

## 🎥 Gazebo Experiment Results



This repository provides the **ROS 2 Jazzy implementation** of the controller proposed in the paper:

> **Hamza Tariq**, Usman Ali, and Adeel Akhtar  
> **“Simultaneous Tracking and Stabilization of a Nonholonomic Robot: A Lipschitz Continuous Quadratic Program based Controller”**, 2026.  

---

Below are the simulation results demonstrating **trajectory tracking** for the proposed **DFL-QP controller**, compared against the **standard DFL controller**.


---

### 🔵 Trajectory Tracking Comparison on Half Figure-8

<table>
<tr>
<td align="center"><b>Proposed DFL-QP Controller</b></td>
<td align="center"><b>Standard DFL Controller</b></td>
</tr>
<tr>
<td><img src="Gazebo_Simulation_Videos/DFL_QP_Tracking.gif" width="400"/></td>
<td><img src="Gazebo_Simulation_Videos/Standard_DFL_Tracking.gif" width="400"/></td>
</tr>
</table>


---

## 🧩 Environment Setup

| Component | Recommended Version |
|------------|--------------------|
| **Ubuntu** | 24.04 LTS |
| **ROS 2** | Jazzy Jalisco |
| **Python** | 3.10 – 3.12 |
| **CMake** | ≥ 3.16 |
| **Gazebo** | Harmonic |

---

## 🧰 Python and ROS Dependencies
```bash
# System and ROS dependencies
sudo apt update
sudo apt install python3-pip python3-numpy python3-scipy \
                 ros-jazzy-rclpy ros-jazzy-geometry-msgs \
                 ros-jazzy-std-msgs ros-jazzy-nav-msgs \
                 ros-jazzy-tf-transformations

# Python optimization and math libraries
pip install qpsolvers osqp scipy numpy
```


## Run the experiments
```bash
git clone https://github.com/gradslab/DFL_QP_Unicycle.git
cd DFL_QP_Unicycle/
colcon build
```
Now in a new terminal 
```bash
source path_to_cloned_repo/DF_QP_Unicycle/install/setup.bash
```

To run trajtory tracking experiment for figure-8 experiment
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml x:=-0.2 y:=0.0 yaw:=3.14159
```

To run our controller
```bash
ros2 run controllers dfl_qp_tracking
```
Alternatively, if you want to run tracking for tradition DFL method run 
```bash
ros2 run controllers std_dfl_tracking
```
