# Autonomous Car Driving System with ROS2 Humble

A repository that is custom made for a custom car-like mobile robot. This is the full Autonomous Car Driving System (ACDS) repository. 

> **Note:** The mobile robot is made for WRO Future Engineer, for hardware details you can find it here:
> [WRO 3ID GitHub Repo](https://github.com/Phumint/wro_3id)
---
### Clone the ACDS Repository
```bash
# Navigate to the home directory, or any other directory you want 
cd ~

# Clone this repository
git clone https://github.com/Phumint/acds_ws.git

# Go to the Workspace directory Build the package
cd ~/acds_ws
colcon build 

# Source your workspace
source install/setup.bash

```
### Start pigpio
```bash
sudo pigpiod
```

### Launch the lane follower feature
```bash
# Source your workspace
cd ~/acds_ws
source install/setup.bash

# Launch the Lane Follower
ros2 launch acds_launch acds.launch.py
```

