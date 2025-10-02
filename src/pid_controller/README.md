# ROS 2 PID Controller and Plant Simulation

This package provides a simple **PID controller node** and a **plant simulation node** to test in C++.  
It is designed to demonstrate proportional-integral-derivative (PID) closed-loop control in ROS 2 using publisher–subscriber communication.

---

## Features
- Configurable **PID gains** (`kp`, `ki`, `kd`) via ROS parameters
- Separate **plant simulation node** with a simple first-order model for testing PID Node
- Real-time feedback loop using ROS topics

---

## Nodes

### 1. PID Controller Node
- **Subscribes:**  
  - `/setpoint` (`std_msgs/Float64`) – desired target  
  - `/measurement` (`std_msgs/Float64`) – current system output  

- **Publishes:**  
  - `/control_output` (`std_msgs/Float64`) – control signal  

- **Parameters:**  
  - `kp` – proportional gain  
  - `ki` – integral gain  
  - `kd` – derivative gain  

---

### 2. Plant Simulation Node
- **Subscribes:**  
  - `/control_output` – control signal from PID  

- **Publishes:**  
  - `/measurement` – simulated system output  

- **Parameters:**  
  - `dt` – integration timestep  
  - `initial_state` – starting condition  

---

## Build Instructions
```bash
cd ~/ros2_ws/src
git clone <repo_url>
cd ~/ros2_ws
colcon build --packages-select pid_controller
source install/setup.bash
