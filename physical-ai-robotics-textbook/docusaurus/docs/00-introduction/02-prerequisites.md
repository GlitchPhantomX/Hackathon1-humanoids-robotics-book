---
sidebar_position: 2
title: "Prerequisites and Setup Requirements"
---

import ReadingTime from '@site/src/components/ReadingTime';
import ViewToggle from '@site/src/components/ViewToggle';

<ReadingTime minutes={12} />
<!-- <ViewToggle /> -->
# <h1 className="main-heading">Prerequisites and Setup Requirements</h1>
<div className="underline-class"></div>

<div className="full-content">

**Module**: 00 - Introduction
**Learning Objectives**:
- • Assess your current skill level against the prerequisites
- • Install and configure the required software stack
- • Set up your development environment for robotics
- • Verify your system meets hardware requirements
<div className="border-line"></div>

**Prerequisites**: Basic programming experience, Linux command line familiarity
**Estimated Time**: 2-4 hours (for initial setup)

<div className="border-line"></div>
---

<h2 className="second-heading">
 Introduction
</h2>
<div className="underline-class"></div>

Before diving into the world of Physical AI and humanoid robotics, you'll need to ensure you have the proper prerequisites and setup. This chapter outlines the knowledge, tools, and hardware requirements you'll need throughout this textbook. Proper setup is crucial for a smooth learning experience, so please follow these instructions carefully.

This chapter covers both the knowledge prerequisites you should possess and the practical setup required to follow along with the hands-on exercises and projects in this textbook.

<div className="border-line"></div>
---

<h2 className="second-heading">
 Knowledge Prerequisites
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
Programming Skills
</h3>
<div className="underline-class"></div>

- ➤ **Python**: Intermediate-level proficiency (functions, classes, modules, exception handling)
- ➤ **Basic C++**: Understanding of object-oriented concepts (helpful for ROS 2)
- ➤ **Linux Command Line**: Comfortable with terminal operations, file systems, and basic commands
- ➤ **Git Version Control**: Understanding of cloning, committing, and pulling code
<div className="border-line"></div>
<h3 className="third-heading">
- Mathematics Foundation
</h3>
<div className="underline-class"></div>

- ➤ **Linear Algebra**: Vectors, matrices, transformations (rotation, translation)
- ➤ **Calculus**: Derivatives and integrals (especially for motion planning)
- ➤ **Probability and Statistics**: For sensor fusion and uncertainty modeling
- ➤ **Basic Physics**: Understanding of forces, motion, and mechanics
<div className="border-line"></div>

<h3 className="third-heading">
- Robotics Concepts (Optional but Helpful)
</h3>
<div className="underline-class"></div>

- ➤ Basic understanding of robot kinematics and dynamics
- ➤ Familiarity with sensors and actuators
- ➤ Knowledge of control systems fundamentals
<div className="border-line"></div>

<div className="border-line"></div>
---

<h2 className="second-heading">
 Software Requirements
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Operating System
</h3>
<div className="underline-class"></div>

- ➤ **Ubuntu 20.04 LTS or 22.04 LTS** (recommended for ROS 2 compatibility)
- ➤ **Alternative**: Windows with WSL2 or macOS (with additional setup complexity)
<div className="border-line"></div>
<h3 className="third-heading">
- Core Software Stack
</h3>
<div className="underline-class"></div>

- ➤ **ROS 2 Humble Hawksbill** or **Rolling Ridley** (latest LTS version)
- ➤ **Python 3.8 or higher**
- ➤ **C++ Compiler** (GCC 9 or higher)
- ➤ **Git** (version 2.25 or higher)
- ➤ **Docker** (for containerized development environments)
<div className="border-line"></div>
<h3 className="third-heading">
- Development Tools
</h3>
<div className="underline-class"></div>

- ➤ **IDE**: VS Code with ROS extension or CLion for C++
- ➤ **Terminal**: GNOME Terminal or equivalent
- ➤ **Build Tools**: CMake, Make, Colcon
- ➤ **Package Managers**: APT, pip, conda

<div className="border-line"></div>
---

<h2 className="second-heading">
 Hardware Recommendations
</h2>
<div className="underline-class"></div>

<h3 className="third-heading">
- Minimum Requirements
</h3>
<div className="underline-class"></div>

- ➤ **CPU**: 4+ cores, 2.5GHz or faster
- ➤ **RAM**: 8GB (16GB recommended)
- ➤ **Storage**: 50GB free space (100GB+ recommended)
- ➤ **Graphics**: Integrated graphics or dedicated GPU (NVIDIA preferred)
<div className="border-line"></div>

<h3 className="third-heading">
- Recommended for Advanced Work
</h3>
<div className="underline-class"></div>

- ➤ **NVIDIA GPU**: RTX series with CUDA support (for Isaac and AI workloads)
- ➤ **Robot Hardware**: Any ROS-compatible robot platform (TurtleBot3, UR5, etc.)
- ➤ **Sensors**: RGB-D camera, IMU, LIDAR (for hands-on projects)
<div className="border-line"></div>

---

<h2 className="second-heading">
 Hands-On Exercises
</h2>
<div className="underline-class"></div>

:::tip Exercise 0.2.1: Environment Setup Verification

**Objective**: Verify that your development environment is properly configured for robotics development.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 20-30 minutes

**Requirements**:
1. Install Ubuntu 20.04/22.04 LTS or set up WSL2 on Windows
2. Install ROS 2 Humble Hawksbill
3. Set up Python and C++ development tools
4. Verify Git and Docker installations

**Starter Code**:
```bash
# Environment verification script template
#!/bin/bash

echo "=== System Environment Verification ==="

echo "1. Checking OS version:"
lsb_release -a

echo "2. Checking Python version:"
python3 --version

echo "3. Checking GCC version:"
gcc --version

echo "4. Checking Git version:"
git --version

echo "5. Checking Docker version:"
docker --version

echo "6. Checking ROS 2 installation:"
if command -v ros2 &> /dev/null; then
    echo "ROS 2 version: $(ros2 --version)"
else
    echo "ROS 2 not found. Please install ROS 2 Humble Hawksbill."
fi
```

**Deliverable**: A verification report showing all required software is properly installed.

**Success Criteria**:
- [ ] `ros2 --version` returns version information (ROS 2 Humble Hawksbill)
- [ ] `python3 --version` shows Python 3.8 or higher
- [ ] `gcc --version` shows GCC 9 or higher
- [ ] `git --version` returns Git version 2.25 or higher
- [ ] `docker --version` returns Docker version

**Test Commands**:
```bash
# Verify all installations
ros2 --version
python3 --version
gcc --version
git --version
docker --version
```

**Expected Output**:
```
ros2 foxy/rolling/humble (version number)
Python 3.x.x
gcc (Ubuntu x.x.x) x.x.x
git version x.x.x
Docker version x.x.x, build xxxxx
```

**Challenge**: Create a shell script that automatically verifies all prerequisites and generates a comprehensive report.

**Hints**:
<details>
<summary>Click for hint 1</summary>

Make sure you have sourced the ROS 2 setup script in your terminal session.
</details>

<details>
<summary>Click for hint 2</summary>

If you're using WSL2 on Windows, ensure you have installed the WSL2 kernel update and configured it properly.
</details>
:::

:::tip Exercise 0.2.2: Basic ROS 2 Workspace Setup

**Objective**: Create and build a basic ROS 2 workspace to verify the installation.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 25-35 minutes

**Requirements**:
1. Create a new ROS 2 workspace directory
2. Initialize the workspace with colcon
3. Build the workspace successfully
4. Source the workspace environment
5. Verify ROS 2 commands work in the workspace context

**Starter Code**:
```bash
# Workspace setup template
#!/bin/bash

# Define workspace name and location
WORKSPACE_NAME="~/ros2_ws"
SOURCE_WS_SCRIPT="install/setup.bash"

echo "Setting up ROS 2 workspace..."

# Create workspace directory structure
mkdir -p $WORKSPACE_NAME/src

# Navigate to workspace
cd $WORKSPACE_NAME

# Build the workspace
colcon build

# Source the workspace
source $SOURCE_WS_SCRIPT

echo "Workspace setup complete!"
echo "Current ROS distribution: $ROS_DISTRO"
echo "Current workspace: $(pwd)"
```

**Deliverable**: A successfully created and built ROS 2 workspace with verification of functionality.

**Success Criteria**:
- [ ] Workspace directory created at `~/ros2_ws`
- [ ] `colcon build` completes without errors
- [ ] Environment variables properly sourced
- [ ] `echo $ROS_DISTRO` shows the correct ROS distribution
- [ ] ROS 2 commands work in the workspace context

**Test Commands**:
```bash
# Create and build workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
echo $ROS_DISTRO
ros2 topic list
```

**Expected Output**:
```
humble
# (topic list should show at least /parameter_events and /rosout)
```

**Challenge**: Set up a secondary workspace and configure ROS 2 to use overlays properly.

**Hints**:
<details>
<summary>Click for hint 1</summary>

The `colcon build` command may take several minutes to complete on the first run.
</details>

<details>
<summary>Click for hint 2</summary>

After sourcing the workspace, the environment variables will only persist for the current terminal session.
</details>
:::

:::tip Exercise 0.2.3: Python Proficiency Assessment

**Objective**: Demonstrate required Python skills for robotics development through a practical exercise.

**Difficulty**: ⭐⭐ Medium

**Time Estimate**: 30-40 minutes

**Requirements**:
1. Create a Python script that implements a basic robot command parser
2. Use classes and functions to represent robot commands
3. Implement error handling for invalid commands
4. Use list comprehension and basic data structures
5. Include proper documentation and comments

**Starter Code**:
```python title="robot_command_parser.py"
#!/usr/bin/env python3
"""
Robot Command Parser - Prerequisites Exercise

This script demonstrates basic Python skills required for robotics development.
Implement a simple robot command parser that can handle basic movement commands.
"""

class RobotCommandParser:
    """A simple robot command parser for the prerequisites assessment."""

    def __init__(self):
        """Initialize the robot command parser."""
        self.supported_commands = [
            'move_forward',
            'move_backward',
            'turn_left',
            'turn_right',
            'stop',
            'get_position'
        ]
        # Initialize robot position
        self.x = 0
        self.y = 0
        self.orientation = 0  # in degrees, 0 = facing right

    def parse_command(self, command_str):
        """
        Parse a command string and execute the corresponding action.

        Args:
            command_str (str): Command to parse

        Returns:
            dict: Result of the command execution
        """
        # TODO: Implement command parsing logic
        pass

    def execute_command(self, command, params=None):
        """
        Execute a validated command.

        Args:
            command (str): Command to execute
            params (dict): Optional parameters for the command

        Returns:
            dict: Result of the execution
        """
        # TODO: Implement command execution logic
        pass

def main():
    """Main function for the robot command parser."""
    parser = RobotCommandParser()

    # TODO: Add command line argument parsing
    # TODO: Implement interactive mode
    # TODO: Add test cases

    print("Robot Command Parser initialized.")
    print(f"Supported commands: {parser.supported_commands}")

if __name__ == "__main__":
    main()
```

**Deliverable**: A fully functional Python script that meets all requirements with proper error handling and documentation.

**Success Criteria**:
- [ ] Script successfully parses different command types
- [ ] Proper use of object-oriented programming principles
- [ ] Appropriate error handling for invalid inputs
- [ ] Clean, well-commented code with docstrings
- [ ] Includes test cases demonstrating functionality

**Test Commands**:
```bash
# Create the Python assessment script
touch ~/ros2_ws/src/robot_command_parser.py
nano ~/ros2_ws/src/robot_command_parser.py
# Run the script to test functionality
python3 ~/ros2_ws/src/robot_command_parser.py
```

**Expected Output**:
A working robot command parser that can handle basic movement commands and report position.

**Challenge**: Extend the parser to handle coordinate-based movement (e.g., move to x, y coordinates).

**Hints**:
<details>
<summary>Click for hint 1</summary>

Use Python's built-in `argparse` module for command line argument parsing.
</details>

<details>
<summary>Click for hint 2</summary>

Consider using a state machine pattern to track robot position and orientation.
</details>
:::

<div className="border-line"></div>
---

<h2 className="second-heading">
 Common Issues and Debugging
</h2>
<div className="underline-class"></div>

:::caution Common Problems

**Problem 1: ROS 2 Installation Issues**

**Symptoms**:
- • Package not found during ROS 2 installation
- • Repository key errors
- • Dependency conflicts during installation
- • APT update fails after adding ROS repository

**Cause**: Incorrect Ubuntu version, outdated repository information, or network connectivity issues.

**Solution**:
```bash
# Verify your Ubuntu version matches the ROS 2 distribution
lsb_release -sc

# Update package lists
sudo apt update

# If repository key errors occur, re-add the key:
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Verify system time is accurate (critical for HTTPS repositories)
timedatectl status
```

**Verification**:
```bash
# Test ROS 2 installation
ros2 --version
```

<div className="border-line"></div>
---

**Problem 2: Permission Issues with ROS**

**Symptoms**:
- • Permission denied when running ROS commands
- • Cannot access serial ports for hardware communication
- • Colcon build fails with permission errors

**Cause**: Running ROS as root or incorrect user group membership.

**Solution**:
```bash
# Do not run ROS commands with sudo
# Instead, add your user to necessary groups:
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

# Log out and log back in for changes to take effect
# Or run this command to refresh group membership:
newgrp dialout
```

**Verification**:
```bash
# Check group membership
groups $USER
# Should include dialout and plugdev
```

<div className="border-line"></div>
---

**Problem 3: Python Package Issues After ROS Installation**

**Symptoms**:
- • Python packages not found after ROS installation
- • Import errors when using ROS Python libraries
- • Mixed Python environments causing conflicts

**Cause**: Not sourcing the ROS environment or conflicting Python installations.

**Solution**:
```bash
# Ensure you've sourced the ROS environment:
source /opt/ros/humble/setup.bash

# Add to your ~/.bashrc file for permanent effect:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# If using Python virtual environments, source ROS before activating venv:
source /opt/ros/humble/setup.bash
python3 -m venv my_robotics_env
source my_robotics_env/bin/activate
```

**Verification**:
```bash
# Test Python ROS import
python3 -c "import rclpy; print('ROS Python libraries available')"
```

<div className="border-line"></div>
---

**Problem 4: Colcon Build Dependencies Issues**

**Symptoms**:
- • Colcon build fails with missing dependencies
- • Package.xml dependencies not resolved
- • Build stops due to missing packages

**Cause**: Missing dependencies or rosdep not configured properly.

**Solution**:
```bash
# Install dependencies using rosdep from workspace root:
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# If rosdep is not installed:
sudo apt install python3-rosdep
sudo rosdep init  # Run once as root
rosdep update     # Run as user
```

**Verification**:
```bash
# Try building again after dependency installation
cd ~/ros2_ws
colcon build
```

<div className="border-line"></div>
---

**Problem 5: Network Communication Issues Between ROS 2 Nodes**

**Symptoms**:
- • Nodes on different machines cannot communicate
- • DDS discovery fails
- • Topic echoing works locally but not remotely

**Cause**: Different ROS_DOMAIN_ID values or firewall blocking DDS traffic.

**Solution**:
```bash
# Check that both machines have the same ROS_DOMAIN_ID:
echo $ROS_DOMAIN_ID

# If not set, set it to a consistent value:
export ROS_DOMAIN_ID=0

# Check firewall settings allow ROS 2 traffic on UDP ports 7400-7500
# On Ubuntu with UFW:
sudo ufw allow from [OTHER_MACHINE_IP] to any port 7400:7500 proto udp
```

**Verification**:
```bash
# Test network connectivity
ping [OTHER_MACHINE_IP]
# Test ROS 2 domain
echo $ROS_DOMAIN_ID
# Test topic discovery across machines
ros2 topic list
```
:::

<div className="border-line"></div>
---

<h2 className="second-heading">
 Summary
</h2>
<div className="underline-class"></div>

In this chapter, you learned:

- ✅ How to assess your current skill level against the prerequisites
- ✅ How to install and configure the required software stack
- ✅ How to set up your development environment for robotics
- ✅ How to verify your system meets hardware requirements
- ✅ Common issues and how to resolve them

**Key Takeaways**:
- • Proper environment setup is crucial for a smooth learning experience
- • The prerequisites ensure you can follow along with hands-on exercises
- • Troubleshooting skills are essential for robotics development
- • Verification of installations prevents issues later in the course

<div className="border-line"></div>
---

<h2 className="second-heading">
 Additional Resources
</h2>
<div className="underline-class"></div>

**Official Documentation**:
- • [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- • [Ubuntu Installation Guide](https://ubuntu.com/tutorials/install-ubuntu-desktop)

**Tutorials**:
- • [Python for Robotics](https://roboticsbackend.com/python-for-robotics/)
- • [Linux Command Line Basics](https://www.linuxcommand.org/lc3_learning_the_shell.php)

**Example Code**:
- • [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

<div className="border-line"></div>
---

<h2 className="second-heading">
 Navigation
</h2>
<div className="underline-class"></div>

[← Previous Chapter](./01-welcome.md) | [Next Chapter →](./03-hardware-requirements.md)

</div>

<div className="summary-content">

## 📝 Chapter Summary

### Key Concepts
- **Prerequisites**: Knowledge (Python, Linux, math), software (ROS 2, development tools), and hardware requirements for robotics development
- **Environment Setup**: Proper installation and configuration of ROS 2, Python, and development tools
- **Verification**: Testing that all components are properly installed and configured
<div className="border-line"></div>

<div className="third-heading">
Essential Code Pattern
</div>

```bash
# ROS 2 Environment Setup Pattern
# 1. Install ROS 2
sudo apt install ros-humble-desktop-full

# 2. Source the setup script
source /opt/ros/humble/setup.bash

# 3. Create workspace
mkdir -p ~/ros2_ws/src

# 4. Build workspace
cd ~/ros2_ws
colcon build

# 5. Source workspace
source install/setup.bash
```
<div className="border-line"></div>
<div className="third-heading">
Quick Reference
</div>
| Component | Requirement | Verification |
|-----------|-------------|--------------|
| Python | 3.8+ | `python3 --version` |
| ROS 2 | Humble Hawksbill | `ros2 --version` |
| GCC | 9+ | `gcc --version` |
| Git | 2.25+ | `git --version` |
| Docker | Latest | `docker --version` |
<div className="border-line"></div>
<div className="third-heading">
What You Built
</div>

- Complete development environment for robotics
- Verified ROS 2 installation
- Created and tested ROS 2 workspace
<div className="border-line"></div>

### Next Steps
Continue to [Hardware Requirements](./03-hardware-requirements.md) to understand the physical components needed for robotics projects.

---

## Navigation

[← Previous Chapter](./01-welcome.md) | [Next Chapter →](./03-hardware-requirements.md)

</div>