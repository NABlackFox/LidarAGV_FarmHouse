# AGV Autonomous Navigation System

This repository contains the source code for the AGV Autonomous Navigation System developed using ROS 2 Jazzy.

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

Since this repository only contains the `src` directory, you must place it inside a ROS 2 workspace before building.

Create a workspace (workspace name can be anything):

```bash
mkdir -p ~/your_workspace/src
cd ~/your_workspace/src
```

Clone the repository:

```bash
git clone <repository-url>
```

Your workspace structure should look similar to:

```text
your_workspace/
├── src/
│   └── <repository-folder>
```

---

# Build Instructions

Navigate to the workspace root:

```bash
cd ~/your_workspace
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

# Environment Setup

Every new terminal session requires sourcing ROS 2 and the workspace.

Source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Source the workspace:

```bash
source ~/your_workspace/install/setup.bash
```

Alternatively, add these commands to your `.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/your_workspace/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
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
source install/setup.bash
```

Launch the LiDAR driver:

```bash
ros2 launch rplidar_ros rplidar.launch.py
```

---

## Verify LiDAR Data

Check available topics:

```bash
ros2 topic list
```

Expected topic:

```text
/scan
```

Monitor scan data:

```bash
ros2 topic echo /scan
```

## Detaching the Device

When finished, detach the USB device from WSL using PowerShell:

```powershell
usbipd detach --busid <BUSID>
```

Example:

```powershell
usbipd detach --busid 2-4
```
---

# Running the System



## Launch the AGV System

```bash
ros2 launch <package_name> <launch_file>.launch.py
```

Example:

```bash
ros2 launch agv_bringup agv.launch.py
```

---

# Visualization with RViz2

Open RViz2:

```bash
rviz2
```

Or launch RViz through the provided launch file:

```bash
ros2 launch <package_name> <rviz_launch_file>.launch.py
```

---

# Mapping (SLAM)

Start the SLAM system:

```bash
ros2 launch <package_name> <slam_launch_file>.launch.py
```

Save the generated map:

```bash
ros2 run nav2_map_server map_saver_cli -f map
```

This will generate:

```text
map.pgm
map.yaml
```

---

# Localization and Navigation

Launch localization using a previously generated map:

```bash
ros2 launch <package_name> <localization_launch_file>.launch.py
```

Launch navigation:

```bash
ros2 launch <package_name> <navigation_launch_file>.launch.py
```

---

# Useful Commands
Create new package, replace `my_package` with your pkg
```bash
ros2 pkg create --build-type ament_cmake my_package
```

List active nodes:

```bash
ros2 node list
```

List active topics:

```bash
ros2 topic list
```

Monitor topic data:

```bash
ros2 topic echo <topic_name>
```

Check TF tree:

```bash
ros2 run tf2_tools view_frames
```

---

# Rebuilding After Changes

After modifying the source code:

```bash
cd ~/your_workspace

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

To rebuild a specific package:

```bash
colcon build --packages-select <package_name>
```

Example:

```bash
colcon build --packages-select agv_navigation
```

---

# Troubleshooting

If ROS 2 cannot find a package:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

If build artifacts need to be cleaned:

```bash
rm -rf build install log
```

Then rebuild:

```bash
colcon build --symlink-install
```

---

# Project Structure

```text
your_workspace/
├── build/
├── install/
├── log/
└── src/
    └── <repository-folder>
```

