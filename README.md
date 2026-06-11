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

This repository is intended to be used as the root of a ROS 2 workspace.

Create the workspace directory:

```bash
mkdir -p ~/ros2_ws
```

Clone the repository directly into the workspace:

```bash
git clone <repository-url> ~/ros2_ws
```

Navigate to the workspace:

```bash
cd ~/ros2_ws
```

Your workspace structure should look similar to:

```text
ros2_ws/
├── README.md
└── src/
    ├── lidar_sensor/
    └── orlar_agv/
```

After building the workspace, the following directories will be generated automatically:

```text
ros2_ws/
├── build/
├── install/
├── log/
├── README.md
└── src/
```

Build the workspace:

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
```

Source the workspace:

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
# Running the AGV Simulation

Before launching the simulation, ensure the Gazebo model path is exported so Gazebo can locate the custom world models:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros2_ws/src/orlar_agv/models
```

Launch the AGV simulation:

```bash
ros2 launch orlar_agv launch_sim.launch.py
```

This launch file starts:

- Gazebo Sim (GZ Sim)
- Robot State Publisher
- ROS 2 Control Controllers
- ROS-Gazebo Bridge
- Joystick Interface
- Twist Multiplexer
- AGV Robot Model
## Teleoperation

To manually control the AGV using the keyboard, open a new terminal and source the workspace:


Run the teleoperation node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

### Controls

```text
u    i    o
j    k    l
m    ,    .
```

- `i` : Move forward
- `,` : Move backward
- `j` : Turn left
- `l` : Turn right
- `k` : Stop

## Changing the Simulation World (Optional)

The simulation world loaded by Gazebo is configured in:

```text
src/orlar_agv/launch/launch_sim.launch.py
```

Locate the following section:

```python
default_world = os.path.join(
    get_package_share_directory(package_name),
    'worlds',
    'orlar_table_world.sdf'
)
```

Replace `orlar_table_world.sdf` with another world file located in the `worlds` directory.

Example:

```python
default_world = os.path.join(
    get_package_share_directory(package_name),
    'worlds',
    'greenhouse_world.sdf'
)
```

After changing the world, rebuild the workspace:

```bash
cd ~/ros2_ws

colcon build --symlink-install

source install/setup.bash
```

Then relaunch the simulation:

```bash
ros2 launch orlar_agv launch_sim.launch.py
```
---
# Slam_tool_box
### Option 1: 

To use the slam_tool_box, run the simulation like the previous section then use this command to enable the slam
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/orlar_agv/config/mapper_params_online_async.yaml use_sim_time:=true
```
To change any setting for the slam_tool_box please modidy the `mapper_params_online_async.yaml` inside the `config` folder

To show it in rviz2 use this command
```bash
ros2 run rviz2 rviz2 -d src/orlar_agv/config/slam_mapping.rviz 
```
### Option 2: All-in-One Quick Run (Unified Launch)

If you want to spin up Gazebo, SLAM, and your custom RViz setup simultaneously without opening multiple terminals, use this single command:
```bash
ros2 launch orlar_agv orlar_mapping.launch.py
```

### Save map
To save the map please input these location accordingly in the slam pannel

`Save map:` ./src/orlar_agv/maps/orlar_farm_house_map

`Serialize map:` ./src/orlar_agv/maps/orlar_farm_house_serial

---

# Robot Model Visualization

Launch the robot model visualization environment:

```bash
ros2 launch orlar_agv view_robot_joint_state_gui.launch.py
```

This launch file starts:

- Robot State Publisher
- RViz2 with the provided configuration
- Joint State Publisher GUI

The AGV model will appear in RViz2 and can be manipulated using the Joint State Publisher GUI.

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

# Use full Command

Run Rviz2 with config
```bash
ros2 run rviz2 rviz2 -d /path/to/your_config.rviz
```

If the pkg build fail but the error is not inside the file, try remove the build, log and install folder then rebuild
```bash
cd ~/ros2_ws
rm -rf build/ log/ install/
colcon build --symlink-install
```