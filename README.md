# Robotic_arm_LTTS

# Implementing Vision-Guided Robotic Pick-and-Place System using ROS 2

An end-to-end autonomous manipulation pipeline: **Perception → TF Transformation → Motion Planning → Control → Simulation** using Franka Emika Panda robot in Gazebo.

The robot detects colored objects (Red, Green, Blue), transforms their positions using TF2, and performs autonomous pick-and-place operations.

---

## Project Overview

This project demonstrates a complete vision-based robotic manipulation system that:

1. Creates Gazebo world and spawns Franka Emika Panda robot
2. Uses camera + OpenCV for real-time color-based object detection
3. Transforms object positions from `camera_link` to `ltts_link0` (robot base) using TF2
4. MoveIt 2 plans collision-aware grasp and place trajectories
5. ros2_control executes the motion in Gazebo simulation
6. Automated pick-and-place for multiple colored objects

The entire pipeline runs **autonomously** once launched.

> **Note**: Pick/grasp fails **not implemented** yet.

---

## Technologies Used

- **ROS 2 Humble**
- **MoveIt 2**
- **Gazebo** (ros-gz)
- **OpenCV** (Python)
- **Franka Emika Panda** Robot
- **ros2_control**
- **TF2**
- **RViz 2**

---

## What I Learned

- Integrating perception with motion planning in ROS 2
- TF frame transformations for vision-guided grasping
- MoveIt 2 planning and trajectory execution
- ros2_control configuration and debugging
- Synchronization between Gazebo, RViz, and MoveIt
- Structuring a complete ROS 2 workspace

---

## How to Run

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (Desktop-Full)
- Gazebo Harmonic

### 1. Install Dependencies

~~~bash
sudo apt update
sudo apt install ros-humble-ros-gz ros-humble-moveit ros-humble-moveit-planners-chomp
sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv python3-opencv
~~~

### 2. Clone the Repository
~~~bash
mkdir ~/ltts_ws/
cd ~/ltts_ws/
git clone https://github.com/Suresh-Robotics/Robotic_Arm_LTTS.git .
~~~
### 3. Build the Workspace

~~~Bash
cd ~/ltts_ws
colcon build --symlink-install
source install/setup.bash
~~~
### 4. Launch the Simulation

~~~Bash
ros2 launch ltts_bringup pick_and_place.launch.py
~~~

### 5. Set Drop Positions (One-time Manual Step)

In RViz2 → Motion Planning panel → Joint tab
Manually move the robot to the desired drop positions for each color
Update the joint values in:

~~~
path is src/pymoveit2/examples/pick_and_place.py

self.red_drop_joints   = [math.radians(0.0), math.radians(-90.0), math.radians(-90.0),
                          math.radians(-4.0), math.radians(-131.0), math.radians(92.0), math.radians(56.0)]
self.green_drop_joints = [...]
self.blue_drop_joints  = [...]
~~~

### 6. Pick the colored object based on priority 
Open a new terminal, source the workspace, and run:
~~~
Python3 automate.py (run in another terminal but source the workspace)

change priority in automate.py file
(Path is ~/ltts_ws/src/pymoveit2/examples/automate.py)

    # Define the sequence you want
    sequence = ["R", "B", "G"]   # Change order or add/remove colors here
~~~



 
 ### -------- Pick-and-place sequence ------------ ###
 1. Move to home joint configuration
 2. Move above target (for pick the mentioned color object)
 3. Open gripper
 4. Move down to approach object
 5. Close gripper
 6. Move to home joint configuration
 7. Move to drop/place joint configuration (here red_drop_joints, green_drop_joints, blue_drop_joints)
 8. Open gripper to release on table
 9. Close gripper
 10. Return to start joint configuration
### ----------------------------------------------------- ###

### Issues I'm facing:
1. Due to my system limitation/computation my robotic arm is not picking the object in sometimes. 
2. So,not able to handle the Failure cases(Pick fails or grasp fails)


> **Note**:  Pick/grasp fails **not implemented** yet.

---



References video:
1. https://www.youtube.com/watch?v=9GeyCgNqoDw
2. https://www.youtube.com/watch?v=0N3B8FG0T1Y
3. Robotic arm,moveit,gazebo,openCV related videos

Reference Links
1. https://github.com/elena-ecn/pick-and-place
2. https://automaticaddison.com/how-to-simulate-a-robotic-arm-in-gazebo-ros-2-jazzy/
3. franka panda robot github links  

