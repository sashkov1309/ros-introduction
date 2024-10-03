# ROS Introduction

## Requirements

- Ubuntu  
  _22.04 LTS preferred_
- Installed ROS Humble
- Clion
- Unix shell understanding  
  _ex. Bash_
- C++ / Python
- Basic cmake

---

## ROS quick description

The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications.   
From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next
robotics project.   
And it's all open source.
---

# ROS Infrastructure

## Checkup of setup

bashrc  
.bashrc is a Bash shell script that Bash runs whenever it is started interactively.

```bash
  nano ~/.bashrc
```

Add in the end of bashrc file

```bash
  source /opt/ros/humble/setup.bash 
  source /home/${USER}/ros_ws/install/local_setup.bash
```
## Package

### Theory

### Creation using CLI

```bash
cd src
ros2 pkg create demo_package --build-type ament_cmake --node-name demo_node 
```

### Writing package contents

---

## Workspace

### Theory

#### File Structure

- src
- build
- install
- log
  Creation using CLI

---

## Build System

Build system of ROS is called **colcon**

### Theory

#### Building existing package with colcon

```bash
sudo apt install python3-colcon-common-extensions
colcon build
```

---

## Launch System

### Theory

---

# Internals of ROS

## Node

### Theory

### Example

### Launch using CLI

```bash
source install/local_setup.bash
ros2 run demo_package demo_publisher
```

### CLI tracing

### Launch using Launch System

### Multinode

---

## Pub/Sub

### Theory

### Topics

### Messages

### Spin

### Example

### CLI tracing

---

## Server/Client

### Theory

### Example

### CLI tracing

---

## Custom messages

### Creation

```bash
cd src
ros2 pkg create demo_dto --build-type ament_cmake
```
String
Int
Bool
Float

---

# Homework

Create package
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
