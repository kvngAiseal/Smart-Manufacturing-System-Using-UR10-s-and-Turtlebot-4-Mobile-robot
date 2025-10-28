# Smart Manufacturing System Using UR10s and Turtlebot 4 Mobile Robot

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A systems integration project demonstrating autonomous material handling in a smart factory environment using a Turtlebot 4 mobile robot and two UR10 collaborative robot arms.

**Authors:** Sela Shapira and Adebolanle Okuboyejo  
**Course:** Robotics Systems Integration

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [System Workflow](#system-workflow)
- [ROS2 Topics](#ros2-topics)
- [Repository Structure](#repository-structure)
- [Demonstration Scenario](#demonstration-scenario)
- [Troubleshooting](#troubleshooting)
- [Future Improvements](#future-improvements)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## 🎯 Overview

This project demonstrates a fully autonomous smart manufacturing system that simulates a mobile device manufacturing facility. The system integrates:

- **1x Turtlebot 4 Mobile Robot** - Autonomous material transport
- **2x UR10 Collaborative Robot Arms** - Pick and place operations

The robots communicate via ROS2 (Robot Operating System 2) topics to coordinate the movement of partially completed components through different manufacturing stages, culminating in the delivery of finished products to storage.

### Key Features

- ✅ Autonomous navigation and material handling
- ✅ Multi-robot coordination using ROS2 publish/subscribe architecture
- ✅ Real-time status communication between robots
- ✅ Modular and scalable system design
- ✅ Simulation of real-world smart factory operations

---

## 🏗️ System Architecture

The system follows a distributed architecture where each robot operates independently while communicating through ROS2 topics:

```
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   UR10 #1   │◄───────►│  Turtlebot 4 │◄───────►│   UR10 #2   │
│  (Pickup)   │         │   (Mobile)   │         │  (Delivery) │
└─────────────┘         └──────────────┘         └─────────────┘
      │                        │                        │
      └────────────────────────┼────────────────────────┘
                               │
                        ROS2 DDS Network
```

---

## 🔧 Hardware Requirements

- **1x Turtlebot 4 Mobile Robot**
  - Onboard computer with ROS2 support
  - Navigation sensors (LiDAR, cameras)
  - Battery power supply

- **2x Universal Robots UR10 Collaborative Arms**
  - End effector/gripper
  - Controller box
  - Network connectivity

- **Computing Infrastructure**
  - PC/Workstation for development and monitoring
  - Reliable WiFi network for robot communication (DDS discovery)

---

## 💻 Software Requirements

### Operating System
- Ubuntu 24.04 LTS (Noble Numbat) - recommended for ROS2 Jazzy

### Core Software
- **ROS2 Jazzy Jalisco**
- **Python 3.10+**
- Turtlebot 4 ROS2 packages
- Universal Robots ROS2 driver

### Python Dependencies
```bash
# Core ROS2 dependencies
rclpy
geometry_msgs
std_msgs
nav_msgs
action_msgs

# Turtlebot 4 specific
turtlebot4_navigation
turtlebot4_msgs

# UR10 specific
ur_robot_driver
ur_msgs
```

---

## 📦 Installation

### 1. Install ROS2 Jazzy

Follow the official ROS2 Jazzy installation guide:
```bash
# Set up sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

### 2. Set Up Your Workspace

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/kvngAiseal/Smart-Manufacturing-System-Using-UR10-s-and-Turtlebot-4-Mobile-robot.git
cd ~/ros2_ws
```

### 4. Install Dependencies

```bash
# Install Turtlebot 4 packages
sudo apt install ros-jazzy-turtlebot4-desktop

# Install UR10 packages
sudo apt install ros-jazzy-ur-robot-driver ros-jazzy-ur-msgs

# Install additional dependencies
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Use rosdep to install any missing dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 6. Configure Network

For ROS2 DDS communication, ensure all robots are on the same network. Add to your `~/.bashrc`:

```bash
# ROS2 Domain ID (use same ID for all robots in the system)
export ROS_DOMAIN_ID=30

# Source workspace
source ~/ros2_ws/install/setup.bash
```

Then source it:
```bash
source ~/.bashrc
```

---

## 🚀 Usage

### Starting the System

#### 1. Launch Turtlebot 4
```bash
# On Turtlebot 4 or its control computer
ros2 launch turtlebot4_navigation nav2.launch.py
```

#### 2. Launch UR10 #1 (Pickup Station)
```bash
# On UR10 #1 control computer go to the project folder and in there
run the following command:
ros2 launch dual_ur10.launch.py
```

#### 3. Launch UR10 #2 (Delivery Station)
```bash
# On UR10 #2 control computer go to the project folder and in there
run the following command:
ros2 launch dual_ur10.launch.py
```

#### 4. Run the Smart Factory Programs
```bash
# Run UR10 #1 pickup program
ros2 run smart_factory ur10_pickup_node

# Run UR10 #2 delivery program
ros2 run smart_factory ur10_delivery_node

# Run Turtlebot coordinator program
ros2 run smart_factory turtlebot_coordinator
```

Or create a launch file to start all nodes:
```bash
ros2 launch smart_factory smart_factory_system.launch.py
```

---

## 🔄 System Workflow

The manufacturing process follows this sequence:

### Stage 1: Pickup Request
1. **UR10 #1** publishes a message requesting Turtlebot to move to pickup position
2. **Turtlebot 4** subscribes to the message and navigates autonomously to UR10 #1

### Stage 2: Component Loading
3. **Turtlebot 4** publishes arrival confirmation upon reaching pickup station
4. **UR10 #1** subscribes to arrival message, picks partially completed component, and places it on Turtlebot
5. **UR10 #1** publishes confirmation that item is loaded

### Stage 3: Transport to Assembly
6. **Turtlebot 4** subscribes to loaded confirmation and navigates to UR10 #2 (delivery station)

### Stage 4: Assembly/Processing
7. **Turtlebot 4** publishes arrival message at delivery station
8. **UR10 #2** subscribes to message, picks up component, performs assembly/processing
9. **UR10 #2** places finished product back on Turtlebot and publishes completion message

### Stage 5: Storage Delivery
10. **Turtlebot 4** subscribes to completion message and navigates to final storage location
11. System ready for next cycle

### Workflow Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                    SMART FACTORY WORKFLOW                     │
└──────────────────────────────────────────────────────────────┘

   UR10 #1 (Pickup)          Turtlebot 4           UR10 #2 (Delivery)
         │                        │                        │
         │───move_request────────>│                        │
         │                        │──navigating...         │
         │<───arrived_at_pickup───│                        │
         │                        │                        │
    [Pick & Place]                │                        │
         │                        │                        │
         │───item_loaded─────────>│                        │
         │                        │──navigating...         │
         │                        │                        │
         │                        │───arrived_at_delivery─>│
         │                        │                        │
         │                        │                   [Pick & Process]
         │                        │                        │
         │                        │<───item_processed──────│
         │                        │                        │
         │                        │──to storage...         │
         │                        │                        │
         │                    [At Storage]                 │
         │                        │                        │
```

---

## 📡 ROS2 Topics

### Published Topics

| Topic Name | Message Type | Publisher | Description |
|-----------|--------------|-----------|-------------|
| `/ur10_1/move_request` | `std_msgs/msg/String` | UR10 #1 | Request for Turtlebot to move to pickup |
| `/turtlebot/arrived_pickup` | `std_msgs/msg/Bool` | Turtlebot 4 | Confirmation of arrival at pickup station |
| `/ur10_1/item_loaded` | `std_msgs/msg/Bool` | UR10 #1 | Confirmation that item is placed on Turtlebot |
| `/turtlebot/arrived_delivery` | `std_msgs/msg/Bool` | Turtlebot 4 | Confirmation of arrival at delivery station |
| `/ur10_2/item_processed` | `std_msgs/msg/Bool` | UR10 #2 | Confirmation that finished product is loaded |

### Subscribed Topics

| Topic Name | Subscriber | Purpose |
|-----------|-----------|---------|
| `/ur10_1/move_request` | Turtlebot 4 | Listen for pickup requests |
| `/turtlebot/arrived_pickup` | UR10 #1 | Know when to start pick operation |
| `/ur10_1/item_loaded` | Turtlebot 4 | Know when to move to delivery |
| `/turtlebot/arrived_delivery` | UR10 #2 | Know when to start assembly operation |
| `/ur10_2/item_processed` | Turtlebot 4 | Know when to move to storage |

### Useful ROS2 Commands

```bash
# List all active topics
ros2 topic list

# Echo messages on a specific topic
ros2 topic echo /ur10_1/move_request

# Get topic information
ros2 topic info /turtlebot/arrived_pickup

# Publish a test message
ros2 topic pub /ur10_1/move_request std_msgs/msg/String "data: 'move'"

# View node graph
ros2 run rqt_graph rqt_graph
```

---

## 📁 Repository Structure

```
Smart-Manufacturing-System-Using-UR10-s-and-Turtlebot-4-Mobile-robot/
├── README.md
├── ur10_pickup_node.py          # UR10 #1 control program
├── ur10_delivery_node.py        # UR10 #2 control program
├── turtlebot_coordinator.py     # Turtlebot coordination program
└── [Additional program files]
```

**Note:** This repository contains the core programs for the smart manufacturing system. Users will need to set up the appropriate ROS2 workspace structure and create launch files as needed for their specific deployment.

### Suggested Workspace Organization

When integrating these programs into your ROS2 workspace, consider this structure:

```
~/ros2_ws/
└── src/
    └── smart_factory/
        ├── package.xml
        ├── setup.py
        ├── smart_factory/
        │   ├── __init__.py
        │   ├── ur10_pickup_node.py
        │   ├── ur10_delivery_node.py
        │   └── turtlebot_coordinator.py
        ├── launch/
        │   └── smart_factory_system.launch.py
        ├── config/
        │   ├── waypoints.yaml
        │   └── ur_poses.yaml
        └── resource/
            └── smart_factory
```

---

## 🏭 Demonstration Scenario

### Mobile Device Manufacturing Simulation

This project simulates a simplified mobile device assembly line:

#### Components:
- **Partially Completed Component**: Represents a semi-assembled mobile device part (e.g., phone chassis with some components)
- **Finished Product**: Represents the completed mobile device ready for packaging

#### Manufacturing Stages:
1. **Initial Assembly Station (UR10 #1)**: Picks partially completed mobile device components
2. **Transport (Turtlebot 4)**: Moves components between stations
3. **Final Assembly Station (UR10 #2)**: Completes assembly and quality check
4. **Storage**: Final products are transported to warehouse area

#### Demo Highlights:
- Zero human intervention required once system is initiated
- Real-time coordination between multiple robots
- Scalable to multiple Turtlebots and workstations
- Demonstrates Industry 4.0 principles

---

## 🔍 Troubleshooting

### Common Issues

#### Robots Not Communicating
```bash
# Check ROS2 domain ID (must be same on all machines)
echo $ROS_DOMAIN_ID

# List all nodes
ros2 node list

# Check if topics are being published
ros2 topic list
ros2 topic hz /ur10_1/move_request

# Monitor specific topic
ros2 topic echo /ur10_1/move_request
```

#### DDS Discovery Issues
```bash
# Check network connectivity
ping <robot_ip>

# Verify DDS discovery
ros2 daemon stop
ros2 daemon start

# Try different DDS implementation (if needed)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# or
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

#### Turtlebot Navigation Issues
- Ensure map is properly loaded
- Check localization: `ros2 launch turtlebot4_viz view_robot.launch.py`
- Verify obstacle detection sensors are functioning

#### UR10 Connection Problems
- Confirm robot IP address is correct
- Check that UR10 is in remote control mode
- Verify network firewall settings allow ROS2 traffic

#### Node Not Running
```bash
# Check if node is active
ros2 node list

# Get node information
ros2 node info /ur10_pickup_node

# Check node logs
ros2 run rqt_console rqt_console
```

#### Message Not Being Received
```bash
# Test message publishing manually
ros2 topic pub /ur10_1/move_request std_msgs/msg/String "{data: 'move'}" --once

# View topic connections
ros2 topic info /ur10_1/move_request -v
```

---

## 🚀 Future Improvements

- [ ] Implement multi-Turtlebot coordination for parallel processing
- [ ] Add computer vision for quality inspection
- [ ] Integrate with MES (Manufacturing Execution System)
- [ ] Implement predictive maintenance monitoring
- [ ] Add support for different product types and routing
- [ ] Develop web-based monitoring dashboard using ROS2 web bridge
- [ ] Implement collision avoidance for multiple mobile robots
- [ ] Add error recovery and fault tolerance mechanisms
- [ ] Integrate RFID/barcode tracking for inventory management
- [ ] Create Gazebo simulation environment for testing
- [ ] Add ROS2 lifecycle nodes for better state management

---

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

Please ensure your code follows ROS2 coding standards and includes appropriate documentation.

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Course Instructors**: Nick Weldin
- **Universal Robots**: UR10 documentation and ROS2 drivers
- **Clearpath Robotics**: Turtlebot 4 platform and documentation
- **ROS2 Community**: Open-source tools and libraries
- **Contributors**: Sela Shapira and Adebolanle Okuboyejo

---

## 📞 Contact

For questions, issues, or collaboration opportunities:

- **GitHub Issues**: [Create an issue](https://github.com/kvngAiseal/Smart-Manufacturing-System-Using-UR10-s-and-Turtlebot-4-Mobile-robot/issues)
- **Project Repository**: [GitHub Link](https://github.com/kvngAiseal/Smart-Manufacturing-System-Using-UR10-s-and-Turtlebot-4-Mobile-robot)

---

## 📚 Additional Resources

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Turtlebot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/)
- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [ROS2 Navigation (Nav2)](https://navigation.ros.org/)
- [ROS2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)

---
