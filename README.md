# autonomous_navigation
Developed an autonomous navigation system using SLAM for LiDAR-based localization, integrating Visual Odometry, Sensor fusion and calibration, Particle filters, and RRT path planning, achieving 92% localization accuracy.

# Grand Prix Autonomous Racing Project

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Testing](#testing)
- [Contributors](#contributors)
- [License](#license)

---

## Introduction
The **Grand Prix Autonomous Racing Project** demonstrates how to build an autonomous racing system that combines advanced robotics algorithms. It uses ROS-Noetic to handle localization, planning, control, and mapping. This project focuses on navigating a race track autonomously with high accuracy and efficiency.

---

## Features
- Adaptive Monte Carlo Localization (AMCL) for robust pose estimation.
- Reactive and global path planning using Gap Follow, RRT, and A* algorithms.
- Hector SLAM for creating environmental maps (bonus feature).
- Real-time control strategies like Pure Pursuit for smooth path-following.

---

## System Architecture
### Localization
- Uses **Laser Scan Matcher** for initial pose estimation.
- Refines localization with **AMCL**.

### Planning and Control
- Reactive Local Planner: Gap Follow or RRT to avoid obstacles.
- Global Planner: A* or Pure Pursuit to navigate the entire track.

### Mapping (Bonus)
- **Hector SLAM** constructs real-time maps using lidar for better planning and navigation.

---

## Installation
Ensure ROS-Noetic is installed. Follow these steps to set up the project:

bash
# Update package lists
sudo apt-get update

# Install required ROS packages
sudo apt-get install ros-noetic-laser-scan-matcher
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-hector-slam

# Clone the project repository
cd ~/catkin_ws/src
git clone <repository_url>
cd ~/catkin_ws

# Build the workspace
catkin_make


---

## Usage
### Step 1: Start the ROS Master
bash
roscore


### Step 2: Launch the Localization Node
bash
roslaunch grand_prix_racing localization.launch


### Step 3: Launch the Planning and Control Node
bash
roslaunch grand_prix_racing planning_control.launch


### Step 4 (Optional): Launch the Mapping Node
bash
roslaunch grand_prix_racing mapping.launch


---

## Configuration
Modify configuration files in the config directory to suit your environment:

- **Localization**: laser_scan_matcher_params.yaml, amcl_params.yaml
- **Planning and Control**: gap_follow_params.yaml, rrt_params.yaml, a_star_params.yaml, pure_pursuit_params.yaml
- **Mapping**: hector_slam_params.yaml

Example YAML snippet for AMCL:
yaml
max_particles: 500
min_particles: 100
kld_err: 0.05
update_min_a: 0.2


---

## Testing
### Simulation
Run tests in a ROS-compatible simulation environment (e.g., Gazebo). Example command:
bash
roslaunch grand_prix_racing simulation.launch


### Real-World Deployment
Test the system on an actual vehicle on a track, ensuring proper calibration of sensors and controllers.

### Metrics for Evaluation
- Lap time
- Obstacle avoidance success rate
- Path-following smoothness

---

## Contributors
- **Your Name** ([GitHub Profile](https://github.com/anuraghruday))

---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

