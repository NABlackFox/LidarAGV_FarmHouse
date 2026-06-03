# AGV Autonomous Navigation System

This repository contains the source code for the AGV Autonomous Navigation System developed using ROS 2 Jazzy.

> **Note:** This guide assumes the ROS 2 workspace is located at `~/ros2_ws`.

## Prerequisites

Before building the project, ensure the following software is installed:

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Colcon
- Git

Verify your ROS 2 installation:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

---

# Workspace Setup

Since this repository only contains the `src` directory, it must be placed inside a ROS 2 workspace.

Create the workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the repository:

```bash
git clone <repository-url>
```

Your workspace structure should look similar to:

```text
ros2_ws/
└── src/
    └── <repository-folder>
```
---

# Environment Setup

Every new terminal session requires sourcing ROS 2 and the workspace.

Source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Alternatively, add these commands to your `.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
---

# Build Instructions

Navigate to the workspace root:

```bash
cd ~/ros2_ws
```

Source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Build all packages:

```bash
colcon build --symlink-install
```

If the build completes successfully, source the workspace:

```bash
source install/setup.bash
```

---

# Robot State Publisher Visualization

Launch the robot state publisher, RViz2, and the Joint State Publisher GUI to visualize and interact with the AGV model.

Start the robot state publisher:

```bash
ros2 launch orlar_agv rsp.launch.py
```

Open RViz2 using the provided configuration file:

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix orlar_agv)/share/orlar_agv/config/rsp_config.rviz
```

Launch the Joint State Publisher GUI to manually adjust joint positions and verify robot kinematics:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
After all three applications are running, the AGV model should appear in RViz2 and respond to joint movements made through the Joint State Publisher GUI.

---

# WSL2 USB Device Setup (LiDAR)

If you are running ROS 2 inside WSL2 and the LiDAR is connected to your Windows machine, you must attach the USB device to WSL before launching the driver.

## Install USBIPD

On Windows, install USBIPD if it is not already available:

```powershell
winget install dorssel.usbipd-win
```

Verify the installation:

```powershell
usbipd --version
```

---

## List Connected USB Devices

Open PowerShell as Administrator and run:

```powershell
usbipd list
```

Example output:

```text
BUSID  VID:PID    DEVICE
2-4    10c4:ea60 Silicon Labs CP210x USB to UART Bridge
```

Locate the LiDAR device and note its BUSID.

---

## Share the Device

Run the following command in PowerShell as Administrator:

```powershell
usbipd bind --busid <BUSID>
```

Example:

```powershell
usbipd bind --busid 2-4
```

This only needs to be done once for a device.

---

## Attach the Device to WSL2

Attach the LiDAR to your ROS environment:

```powershell
usbipd attach --wsl --busid <BUSID>
```

Example:

```powershell
usbipd attach --wsl --busid 2-4
```

---

## Verify the Device in WSL

Inside Ubuntu/WSL:

```bash
ls /dev/ttyUSB*
```

Expected output:

```text
/dev/ttyUSB0
```

You can also verify with:

```bash
dmesg | tail
```

or

```bash
lsusb
```

---

## Configure Serial Port Permissions

If access is denied:

```bash
sudo chmod 666 /dev/ttyUSB0
```

Or add your user to the dialout group:

```bash
sudo usermod -a -G dialout $USER
```

Log out and reopen the terminal afterwards.

---

# Running the RPLiDAR

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

Launch the LiDAR driver:

```bash
ros2 launch rplidar_ros rplidar.launch.py
```

---

# Rebuilding After Changes

After modifying the source code:

```bash
cd ~/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

# Project Structure

```text
ros2_ws/
├── build/
├── install/
├── log/
└── src/
    └── <repository-folder>
```