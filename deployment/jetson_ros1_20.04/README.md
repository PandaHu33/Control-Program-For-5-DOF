# Jetson Ubuntu 18.04 ROS1 Melodic Bringup Scripts

These scripts are called by `control_ui/bridge.py` through SSH.

ROS Melodic is the ROS1 target for Ubuntu 18.04.

## Required Jetson-side components

For the H5 Demo Console to connect normally, Jetson must run:

1. `roscore`
2. CAN communication ROS node
3. Arm low-level control ROS node
4. H5 UDP bridge ROS node for H5 command forwarding and arm telemetry

In the field setup, closing the H5 supervisor or clicking system stop calls
`stop_arm.sh`. That script stops the ROS processes launched by `start_arm.sh`,
including `h5_udp_bridge`, `arm_control`, `can_node`, and `roscore` by default.

The H5 backend expects:

```text
/home/night/robot/start_arm.sh
/home/night/robot/stop_arm.sh
/home/night/robot/check_arm.sh
```

`check_arm.sh` must print these exact lines so the Windows backend can parse status:

```text
CAN_NODE:RUNNING
ARM_CONTROL:RUNNING
```

or:

```text
CAN_NODE:STOPPED
ARM_CONTROL:STOPPED
```

## Install on Jetson

Copy this folder to Jetson:

```bash
mkdir -p /home/night/robot
cp start_arm.sh stop_arm.sh check_arm.sh robot_env.sh /home/night/robot/
chmod +x /home/night/robot/*.sh
```

Copy the H5 UDP bridge package template into your catkin workspace:

```bash
cp -r h5_udp_bridge /home/night/DataSet_ws/src/
cd /home/night/DataSet_ws
catkin_make
source devel/setup.bash
chmod +x src/h5_udp_bridge/scripts/h5_udp_bridge_node.py
```

Edit `/home/night/robot/robot_env.sh` and replace:

```bash
ROBOT_WS=/home/night/DataSet_ws
ROS_MASTER_URI=http://192.168.1.35:11311
ROS_IP=192.168.1.35
CAN_CMD="roslaunch mainpulator socketcan.launch"
ARM_CMD="roslaunch mainpulator mainpulatorlaunch.launch"
UDP_BRIDGE_ENABLE=1
UDP_BRIDGE_CMD="roslaunch h5_udp_bridge h5_udp_bridge.launch"
```

with your real package, launch file, and workspace.

The 100 Hz joint planner and motion CSV logger are both disabled by default.
Enable them for a planned-data run with a fresh output path:

```bash
export ARM_TRAJECTORY_PLANNER_ENABLE=1
export ARM_MOTION_DATA_LOG_ENABLE=1
export ARM_MOTION_DATA_LOG_PATH=/home/night/robot/logs/arm_motion_planned.csv
/home/night/robot/start_arm.sh
```

For the unplanned A/B baseline, leave `ARM_TRAJECTORY_PLANNER_ENABLE=0` and
use a different `ARM_MOTION_DATA_LOG_PATH`. Restart the arm node after changing
either switch; they are startup-only settings.

Persistent Jetson stdout/stderr logs are disabled by default. The start script
redirects launched process output to `/dev/null`, so it will not fill
`/home/night/robot/logs` or a copied `tobot` folder during normal operation.
When file logs are disabled, ROS internal launch logs are redirected to
`/tmp/robot_ros_logs` instead of the working folder.
When debugging on site, temporarily enable files with:

```bash
ROBOT_LOG_ENABLE=1 /home/night/robot/start_arm.sh
```

For the template bridge package, keep:

```bash
UDP_BRIDGE_ENABLE=1
UDP_BRIDGE_CMD="roslaunch h5_udp_bridge h5_udp_bridge.launch"
```

## Jetson packages

On a clean Ubuntu 18.04 Jetson, configure the ROS Melodic apt source first:

```bash
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
```

Minimal packages for SSH, ROS launch, and the UDP bridge:

```bash
sudo apt update
sudo apt install -y openssh-server net-tools iproute2 iputils-ping tcpdump socat can-utils
sudo apt install -y ros-melodic-ros-base ros-melodic-rospy ros-melodic-std-msgs
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential
sudo systemctl enable --now ssh
```

CAN integration depends on the real arm driver. Common options are:

```bash
sudo apt install -y ros-melodic-can-msgs ros-melodic-ros-canopen
```

Install those only if your CAN node uses them.

## Manual test on Jetson

```bash
/home/night/robot/start_arm.sh
/home/night/robot/check_arm.sh
/home/night/robot/stop_arm.sh
```

`stop_arm.sh` stops the processes launched by `start_arm.sh` by default. For
bench debugging only, keep `roscore` running with:

```bash
KEEP_ROSCORE=1 /home/night/robot/stop_arm.sh
```

## SSH test from Windows backend machine

```powershell
ssh night@192.168.1.35 /home/night/robot/check_arm.sh
```

If passwordless SSH is not configured, the H5 backend may block or fail. Configure SSH key login for demo use.

The backend uses non-interactive SSH. This exact command must work from Windows:

```powershell
ssh -o BatchMode=yes -o ConnectTimeout=5 night@192.168.1.35 "/home/night/robot/check_arm.sh"
```

Expected output includes:

```text
CAN_NODE:RUNNING
ARM_CONTROL:RUNNING
```

or the corresponding `STOPPED` lines.

## H5 UDP bridge topics

The package `h5_udp_bridge` listens for H5 binary UDP frames on Jetson
UDP `14551`, verifies CRC32, and publishes parsed commands as JSON:

```text
/h5/arm_command  std_msgs/String
/h5/udp_status   std_msgs/String
/arm/active_control_source  std_msgs/String (latched)
/h5/pub_joint_state  sensor_msgs/Imu
```

`bridge:source:<mode>` frames update `/arm/active_control_source` without
publishing a motion command. The arm controller must keep H5 commands,
external `/pub_joint_state` teleoperation, and
`/imitation/desired_trajectory` as separate inputs and accept only the source
named by `/arm/active_control_source`. Unknown or inactive sources fail closed.

The deployed `mainpulator/src/test_node.cpp` must match the repository version.
It treats ROS command angles as logical coordinates and applies the configured
J1 direction (`logical J1 = -physical J1`) only at the hardware boundary.

The real arm control node should subscribe to `/h5/arm_command`, validate the
mode/order fields again, and then convert them into its own topic/service/action
API. Keep emergency-stop handling in the low-level control path too.

Telemetry can be sent back to Windows by publishing JSON to:

```text
/arm/h5_telemetry std_msgs/String
```

Example telemetry JSON:

```json
{"angle":[0,0,0,0,0,0,0],"current":[0,0,0,0,0,0,0],"torque":[0,0,0,0,0,0,0],"pose_ee":[0,0,0,0,0,0],"pose_elbow":[0,0,0,0,0,0],"status":1,"note":"ready"}
```

## UDP test on Jetson

In three terminals:

```bash
roslaunch h5_udp_bridge h5_udp_bridge.launch
```

```bash
rostopic echo /h5/arm_command
```

```bash
sudo tcpdump -ni any udp port 14551 or udp port 14550
```

Then connect the H5 page to `ws://localhost:8080/udp` and press a keyboard
control key. You should see UDP packets in `tcpdump` and parsed JSON on
`/h5/arm_command`.

## Network mapping

`control_ui/config.yaml` must match the Jetson network:

```yaml
jetson:
  host: "192.168.1.35"
  user: "night"

udp:
  host: "192.168.1.35"
  send_port: 14551
  recv_port: 14550
```

Meaning:

- Windows backend sends H5 command frames to Jetson UDP `14551`.
- Jetson sends arm telemetry back to Windows backend UDP `14550`.
