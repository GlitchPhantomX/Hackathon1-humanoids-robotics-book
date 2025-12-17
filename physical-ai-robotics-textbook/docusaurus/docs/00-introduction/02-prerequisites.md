---
sidebar_position: 2
title: "Prerequisites and Setup Requirements"
---

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={3} />

<h1 className="main-heading">Prerequisites and Setup Requirements</h1>
<div className="underline-class"></div>

**Learning Objectives**:
- • Assess skill level against prerequisites
- • Install required software stack
- • Set up development environment
- • Verify system requirements

**Prerequisites**: Basic programming, Linux CLI | **Time**: 2-4 hours

<div className="border-line"></div>

<h2 className="second-heading">Prerequisites Overview</h2>
<div className="underline-class"></div>

Essential requirements for Physical AI and humanoid robotics development.

<div className="border-line"></div>

<h2 className="second-heading">Knowledge Prerequisites</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Programming Skills</h3>
<div className="underline-class"></div>

- **Python**: Intermediate (functions, classes, modules)
- **C++**: Basic OOP
- **Linux CLI**: Terminal, file systems
- **Git**: Clone, commit, pull

<h3 className="third-heading">Mathematics Foundation</h3>
<div className="underline-class"></div>

- **Linear Algebra**: Vectors, matrices, transformations
- **Calculus**: Derivatives, integrals
- **Probability**: Sensor fusion
- **Physics**: Forces, motion, mechanics

<div className="border-line"></div>

<h2 className="second-heading">Software Requirements</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Operating System</h3>
<div className="underline-class"></div>

- **Ubuntu 20.04/22.04 LTS** (recommended)
- **Alternatives**: WSL2 on Windows, macOS

<h3 className="third-heading">Core Software Stack</h3>
<div className="underline-class"></div>

- **ROS 2**: Humble Hawksbill or Rolling
- **Python**: 3.8+
- **C++ Compiler**: GCC 9+
- **Git**: 2.25+
- **Docker**: For containers

<h3 className="third-heading">Development Tools</h3>
<div className="underline-class"></div>

- **IDE**: VS Code with ROS extension or CLion
- **Build Tools**: CMake, Make, Colcon
- **Package Managers**: APT, pip, conda

<div className="border-line"></div>

<h2 className="second-heading">Hardware Recommendations</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Minimum Requirements</h3>
<div className="underline-class"></div>

- **CPU**: 4+ cores, 2.5GHz+
- **RAM**: 8GB (16GB recommended)
- **Storage**: 50GB+ free
- **Graphics**: Integrated or NVIDIA GPU

<h3 className="third-heading">Advanced Work</h3>
<div className="underline-class"></div>

- **NVIDIA GPU**: RTX series with CUDA
- **Robot Platform**: ROS-compatible (TurtleBot3, UR5)
- **Sensors**: RGB-D camera, IMU, LIDAR

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

**Exercise 0.2.1**: Environment Setup Verification
- Install Ubuntu 20.04/22.04 or WSL2
- Install ROS 2 Humble
- Set up Python and C++ tools
- Verify Git and Docker

**Exercise 0.2.2**: Basic ROS 2 Workspace Setup
- Create workspace directory
- Initialize with colcon
- Build workspace
- Source environment

**Exercise 0.2.3**: Python Proficiency Assessment (⭐⭐, 30-40 min)
- Create robot command parser
- Use classes and functions
- Implement error handling
- Include documentation

<div className="border-line"></div>

<h2 className="second-heading">Common Issues</h2>
<div className="underline-class"></div>

**ROS 2 Installation Issues**:
```bash
lsb_release -sc  # Verify Ubuntu version
sudo apt update
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
ros2 --version  # Verify installation
```

**Permission Issues**:
```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
newgrp dialout
groups $USER  # Verify groups
```

**Python Package Issues**:
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
python3 -c "import rclpy; print('ROS Python libraries available')"
```

**Colcon Build Dependencies**:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

**Network Communication Issues**:
```bash
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0
sudo ufw allow from [OTHER_IP] to any port 7400:7500 proto udp
ros2 topic list
```

<div className="border-line"></div>

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

Proper environment setup is crucial. Prerequisites ensure smooth learning. Troubleshooting skills essential for robotics development.

**Key Takeaways**:
- • Environment setup crucial
- • Prerequisites ensure smooth progress
- • Troubleshooting skills essential
- • Verification prevents later issues

<h2 className="second-heading">Resources</h2>
<div className="underline-class"></div>

- • [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- • [Python for Robotics](https://roboticsbackend.com/python-for-robotics/)
- • [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

**Navigation**: [← Previous](./01-welcome.md) | [Next →](./03-hardware-requirements.md)

<h2 className="second-heading">Quick Reference</h2>
<div className="underline-class"></div>
```bash
# ROS 2 Environment Setup
sudo apt install ros-humble-desktop-full
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

| Component | Requirement | Verification |
|-----------|-------------|--------------|
| Python | 3.8+ | `python3 --version` |
| ROS 2 | Humble | `ros2 --version` |
| GCC | 9+ | `gcc --version` |
| Git | 2.25+ | `git --version` |
| Docker | Latest | `docker --version` |