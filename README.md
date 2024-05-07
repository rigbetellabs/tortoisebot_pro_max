
# Tortoisebot Pro-MAX 
## ROS1 Noetic Release

![tortoisebotpromax Logo](img/logo.png)

<div align="center">

Welcome to the official public repository for **TortoisebotProMax** by **RigBetel Labs**.

**Purpose:**  
This repository hosts essential documentation and code for *TortoisebotProMax Robot*, facilitating transparency and collaboration.

**Privacy:**  
Certain sensitive packages and scripts have been excluded to maintain privacy standards.

**Contents:**  
- **Documentation:** Detailed guides and technical specifications.
- **Codebase:** Essential source code for *Cepheus Robot*.
- **Resources:** Supplementary materials and dependencies.


**Contact:**  
For inquiries and collaboration opportunities, reach out to RigBetel Labs.


<a href="https://rigbetellabs.com/">![Website](https://img.shields.io/website?down_color=lightgrey&down_message=offline&label=Rigbetellabs%20Website&style=for-the-badge&up_color=green&up_message=online&url=https%3A%2F%2Frigbetellabs.com%2F)</a>
<a href="https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA">![Youtube Subscribers](https://img.shields.io/youtube/channel/subscribers/UCfIX89y8OvDIbEFZAAciHEA?label=YT%20Subscribers&style=for-the-badge)</a>
<a href="https://www.instagram.com/rigbetellabs/">![Instagram](https://img.shields.io/badge/Follow_on-Instagram-pink?style=for-the-badge&logo=appveyor?label=Instagram)</a>


</div>

--- 

## Table of Contents
- [**1. Installation**](#1-installation)
- [**2. Connection**](#2-connection)
- [**3. Package Description**](#3-package-description)
   - [**3.1 tortoisebotpromax_description**](#31-tortoisebotpromax_description)
   - [**3.2 tortoisebotpromax_firmware**](#32-tortoisebotpromax_firmware)
   - [**3.3 tortoisebotpromax_gazebo**](#33-tortoisebotpromax_gazebo)
   - [**3.4 tortoisebotpromax_navigation**](#34-tortoisebotpromax_navigation)
   - [**3.5 tortoisebotpromax_odometry**](#35-tortoisebotpromax_odometry)
   - [**3.6 tortoisebotpromax_slam**](#36-tortoisebotpromax_slam)
- [**4. Launch Sequence**](#4-launch-sequence)
   - [**4.1 Map Generation**](#41-map-generation)
   - [**4.2 Autonomous Navigation**](#42-autonomous-navigation-in-the-saved-map)
- [**5. Important Low level Topics**](#5-low-level-ros-topics)
- [**6. tortoisebotpromax Robot Parameters**](#6-tortoisebotpromax-robot-parameters)
- [**7. Diagnostic Tests**](#7-diagnostic-tests)
- [**8. Joystick Control Instructions**](#8-joystick-control-instructions)

## 1. Installation

```bash
cd ~/ros1_ws/src  # Assuming ros1_ws is the name of the workspace
```

Clone the repository into your workspace:

```bash
git clone https://github.com/rigbetellabs/tortoisebot_pro_max.git
```

Install dependent packages:

```bash
cd ~/ros1_ws/src/tortoisebot_pro_max
cat requirements.txt | xargs sudo apt-get install -y 
```

> [!NOTE]
> Check if you already have the ydlidar packages installed; if not, get the packages from repos below.

```bash
cd ~/ros1_ws/src/
git clone https://github.com/rigbetellabs/ydlidar_ros.git
```

> [!NOTE]
> Custom joystick control script currently runs on the robot, enabling waypoint storage and navigation through joy buttons. This node can be accessed on:

```bash
cd ~/ros1_ws/src/
git clone https://github.com/rigbetellabs/joy_with_waypoint_nav.git
```

Build the workspace:

```bash
cd ~/ros1_ws
catkin_make
```  

## 2. Connection

### Initial Wifi Setup

> [!NOTE]
> By default, the robot is programmed to be started up automatically upon bootup, with its ros running locally without the need for any wifi network.

Follow the steps below to connect the robot to your desired Wifi network
#### 1. Create a mobile hotspot
Initiate a hotspot from your smartphone/laptop with the credentials
- Hotspot Name:  `admin`
- Hotspot Password: `adminadmin`

<p align="center">
<img src="img/mobilehotspot.jpeg" width="250"/>
</p>

#### 2. Start the robot
Power on the robot and wait until it connects to your hotspot network

| On powering on:            | When connected to hotspot:                               | 
|--------------------|---------------------------------------------|
| ![Step1](img/booting.gif)   | ![Step2](img/admin.gif)   |

#### 3. SSH into the robot

- Connect your laptop/remote device to the same hotspot
<p align="center">
<img src="img/laptopconnect.png" width="750"/>
</p>

- Open a new terminal, and enter the SSH credentials
```bash
ssh "your-robot-name"@"your-robot-ip"  
pwd: "your-robot-password"
```
> [!TIP]
> The robot name and password have been provided to you while deployment, they have also been marked on the PC present inside the robot. IP can be seen on the display of robot once connected


| Method1           | Method2                            | 
|--------------------|---------------------------------------------|
| ![Step1](img/adminssh.jpeg)   | ![Step1](img/adminssh2.jpeg)    |

#### 4. Connect to Wifi

- Enter the following command on the ssh terminal to check available networks
```bash
sudo nmcli dev wifi list --rescan yes
```
![Step1](img/wifilist.jpeg) 

- Connect to your wifi network
```bash
sudo nmcli device wifi connect "your-wifi-name" password "your-wifi-password"
```
![Step1](img/wificonnect.jpeg) 

> [!IMPORTANT]
> This will close the ssh pipeline and no response will be recieved over it further. Wait for about 30 seconds for robot to be connected to your wifi, once connected it will show the wifi name along with the IP address on the robot display.

#### 4. SSH using your Wifi
- Now the robot is connected to your Wifi network! You can now shutdown your mobile hotspot, connect your remove device to the same wifi and access the robot using SSH:

![Step1](img/wifissh.jpeg) 

To use your PC as a Slave Device to your Cepheus:

```bash
nano ~/.bashrc
export ROS_MASTER_URI=http://"ip address of the robot":11311
export ROS_IP="ip address of your pc"
```

### USB ports Configuration
> [!IMPORTANT]
> Make sure if you dissasemble the robot, reconnect the USB ports as per the following diagram:

![USB Port Connections](img/port_connections.png)

## 3. Package Description

### 3.1 tortoisebotpromax_description

Holds the robot description including URDF, STL, config files for RVIZ, and Gazebo.

| File               | Description                                 | Nodes Launched                    |
|--------------------|---------------------------------------------|-----------------------------------|
| [`display.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_description/launch/display.launch)   | Visualize the URDF of the robot in RVIZ.    | RVIZ, robot_state_publisher, joint_state_publisher      |
| [`gazebo.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_description/launch/gazebo.launch)    | Visualize the Robot in an empty world within Gazebo. | Gazebo, robot_state_publisher    |

### 3.2 tortoisebotpromax_firmware

Provides sensor and actuation topics.

| File                | Description                                             | Nodes Launched                |
|---------------------|---------------------------------------------------------|-------------------------------|
| [`bringup.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_firmware/launch/bringup.launch)    | Brings up all the sensors and actuators on the robot | robot_state_publisher, joint_state_publisher, serial_node, joy, auto_joy_teleop, lidar_node |
| [`demo_mode.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_firmware/launch/demo_mode.launch)       | Single launch file to start all nodes for mapping and navigation, accepts waypoints via joystick control. | brings up bringup.launch, server_bringup.launch and tortoisebotpromax_navigation.launch all at once |
| [`ydlidar_s2.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_firmware/launch/ydlidar_s2.launch) | Launch file for YDLIDAR X2 lidar configuration.        | ydlidar_node |
| [`odom_pub.py`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_firmware/scripts/odom_pub.py)       | Publishes odometry from TF published by Cartographer.   | odom_publisher                           |
| [`network_pub`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_firmware/scripts/network_pub.py)       | Retrieves and publishes the Wifi/Hotspot network data   | network_status                           |
| [`send_goalstatus`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_firmware/scripts/send_goalstatus.py)       | Provides navigation status to the microcontroller   | nav_status                           |

### 3.3 tortoisebotpromax_gazebo
Simulation environment for tortoisebotpromax in Gazebo.

| File                | Description                                             | Nodes Launched                |
|---------------------|---------------------------------------------------------|-------------------------------|
| [`tortoisebotpromax_empty_world.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_gazebo/launch/tortoisebotpromax_empty_world.launch)  | Launch robot in gazebo with an empty world       | robot_state_publisher, gazebo_ros |
| [`tortoisebotpromax_playground.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_gazebo/launch/tortoisebotpromax_playground.launch)  | Launch robot in gazebo with an office simulation world      | robot_state_publisher, gazebo_ros |

### 3.4 tortoisebotpromax_navigation
Autonomous navigation of the robot using `move_base` in a known as well as an unknown environment.

| File                         | Description                              | Nodes Launched                              |
|------------------------------|------------------------------------------|------------------------------------------------------|
| [`amcl.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_navigation/launch/amcl.launch)                | Localize the robot in the environment.   | amcl                                              |
| [`move_base.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_navigation/launch/move_base.launch)           | Node responsible for moving the robot, avoiding obstacles. | move_base                   |
| [`carto_navigation.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_navigation/launch/carto_navigation.launch) | Launches `move_base` with pre-saved or online-generated map using cartographer. | cartographer_occupancy_grid_node, map_server, rviz, move_base      |
| [`gmap_amcl.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_navigation/launch/gmap_amcl.launch) | Launches `move_base` with pre-saved or online-generated map using gmapping and amcl. | gmapping, map_server, rviz, amcl, move_base      |
### 3.5 tortoisebotpromax_odometry
Rtabmap, Ekf and Cartographer based odometry packages.

| File                    | Description                                             | Additional Information                          |
|-------------------------|---------------------------------------------------------|--------------------------------------------------|
| [`carto_odometry.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_odometry/launch/carto_odometry.launch) | Launches Cartographer node for lidar-based odometry.   | cartographer_ros, cartographer                   |
| [`icp_fuse.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_odometry/launch/icp_fuse.launch) | Launches Rtabmap-ICP and EKF nodes for lidar and IMU fused odometry.   | icp_odometry, ekf_localization_node, alpha_beta_filter                  | 

### 3.6 tortoisebotpromax_slam
Simultaneous Localization and Mapping (SLAM) for the robot.

| File                    | Description                                             | Additional Information                          |
|-------------------------|---------------------------------------------------------|--------------------------------------------------|
| [`carto_slam`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_slam/launch/carto_slam.launch)          | Generates a map of the environment using Cartographer.  | cartographer_occupancy_grid_node                                             |
| [`gmapping_slam`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_slam/launch/gmapping_slam.launch)          | Generates a map of the environment using Cartographer.  | gmapping_node                                             |
| [`map_saver.launch`](https://github.com/rigbetellabs/tortoisebotpromax/blob/noetic-release/tortoisebotpromax_slam/launch/map_saver.launch)      | Saves the generated map for navigation.                 | map_server                                            |


## 4. Launch Sequence
> [!NOTE]
> By default, the robot is programmed to be started up automatically upon bootup, with its ros running locally without the need for any wifi network. To get into the development mode of the robot, ssh into the robot and run

```bash
cd ~/ros1_ws/src/cepheus
./development.sh
```
This will stop all your local ros servers permanently and allow you to test your launch files according to will. If you need the robot to be upstart upon bootup again, you can always enable it using
```bash
cd ~/ros1_ws/src/cepheus
./demo.sh
```


### Simulation

```bash
roslaunch tortoisebotpromax_gazebo tortoisebotpromax_playground.launch
```
The gazebo world looks like this:
![playground](img/playground.png)

### Real Robot
#### (Make sure you are in development mode before manually launching the following scripts)


```bash
roslaunch tortoisebotpromax_firmware bringup.launch joy:=true # Set true to get joystick control
```
For cartographer-based odometry:
```bash
roslaunch tortoisebotpromax_odometry carto_odometry.launch 
```
For rtabmap and ekf based odometry:
```bash
roslaunch tortoisebotpromax_odometry icp_fuse.launch 
```
### 4.1 Map Generation
For mapping with manual control:

```bash
roslaunch tortoisebotpromax_slam tortoisebotpromax_slam.launch 
```

For mapping with autonomous navigation:

Using Cartographer:
```bash
roslaunch tortoisebotpromax_navigation tortoisebotpromax_carto_navigation.launch exploration:=true 
```
Using Gmapping:
```bash
roslaunch tortoisebotpromax_navigation tortoisebotpromax_gmap_amcl.launch exploration:=true 
```

To save the map:

```bash
roslaunch tortoisebotpromax_slam map_saver.launch 
```

### 4.2 Autonomous Navigation in the Saved Map

Using Cartographer:
```bash
roslaunch tortoisebotpromax_navigation tortoisebotpromax_carto_navigation.launch exploration:=false map_file:=your_map
```
Using AMCL:
```bash
roslaunch tortoisebotpromax_navigation tortoisebotpromax_gmap_amcl.launch exploration:=false map_file:=your_map
```

> [!NOTE]
> Upon powering on the robot you'll be able to see the bootup animation on the robot

![bootup](img/bootup2.gif) 
<!-- ![bootup2](img/bootup1.gif) -->

> [!NOTE]
> Once the robot is booted up and bringup.launch is initiated, you'll see the Lidar rotating at a higher rate.

## 5. Low-Level ROS Topics

#### `/battery/percentage`
This topic provides information about the remaining battery percentage of the robot. 

| Battery Percentage  | Beeping Sounds              |
|----------------------|-----------------------------|
| 100 - 20            | No beeping                  |
| 20 - 15              | Beep every 2 minutes        |
| 15 - 10              | Beep every 1 minute        |
| Below 10             | Very frequent beeping      |
| 0 (Complete Discharge)| Continuous beep             |

> [!TIP]
> To ensure you are aware of the robot's battery status, pay attention to the beeping sounds, especially as the battery percentage decreases.

> [!CAUTION]
> Do not drain the battery below `10 %`, doing so can damage the battery permanently.

#### `/battery/voltage`
This topic reports the current battery voltage, ranging from 25.2V at maximum charge to 19.8V at minimum charge.

#### `/cmd_vel`
The `/cmd_vel` topic is responsible for receiving velocity commands for the robot. These commands can be generated by teleoperation or the `move_base` module, instructing the robot on how fast to move in different directions.

#### `/pid/control`
This topic is of type `int` and is used to control the Proportional-Integral-Derivative (PID) controller. Publishing `0` stops PID control, `1` starts fast PID control, `2` activates smooth PID control, `3` activate supersmooth PID control.
Here's an example:
```bash
rostopic pub -1 /pid/control std_msgs/Int32 "data: 1"
```
#### `/diagnostics/test`
The `/diagnostics/test` topic is utilized to run diagnostics on the robot. It serves the purpose of identifying and addressing any issues that may arise during the robot's operation. For detailed diagnostics procedures, refer to the documentation.

#### `/wheel/ticks`
This topic provides an array of ticks for all four wheels of the robot, in the format `[lf, lb, rf, rb]`. These values represent the encoder readings of the wheel ticks.

#### `/wheel/vel`
The `/wheel/vel` topic sends an array of calculated velocities for each wheel on the robot, received via encoders. The format of the array is `[lf, lb, rf, rb]`, representing the actual velocity at which each wheel is moving.

## 6. tortoisebotpromax Robot Parameters

| Parameter                   | Value                                     |
|-----------------------------|-------------------------------------------|
| **Wheels Type**             | Differential Wheels                       |
| **Diameter**                | 0.1m                                      |
| **Wheel Separation**        | 0.5m                                      |
| **Motor Type**              | Planetary DC Geared Motor                 |
| **RPM**                     | 100                                       |
| **Encoder Type**            | Magnetic Encoder                          |
| **PPR (Pulses Per Revolution)**| 498                                      |
| **Microcontroller**         | DOIT-ESP32 Devkit V1                      |
| **PC Used**                 | Intel NUC i3 10th Gen                     |
| **Robot Payload Capacity**  | 100 kgs                                   |
| **Battery Life**            | About 3 hours                             |
| **Battery Type**            | Lithium-ion 6-cell, 22.2V                 |

## 7. Diagnostic Tests

### Overview

The diagnostic tests are designed to ensure the proper functioning of various components of the tortoisebotpromax robot. These tests cover motor and encoder connections, motor direction, IMU connections, display connections, and a comprehensive full diagnostic test.

### Instructions

Here is a table summarizing the instructions for each diagnostic test:

| Test Number | Test Type                    |
|-------------|------------------------------|
| 0           | Full Diagnostic Test         |
| 1           | Motor and Encoder Test       |
| 2           | Motor Direction Test         |
| 3           | IMU Connections Test         |
| 4           | Display Connections Test     |

### Detailed Instructions

1. **Full Diagnostic Test (Test Number: 0):**
   - Run the full diagnostic test to check the overall health of the robot.

2. **Motor and Encoder Test (Test Number: 1):**
   - Check motor and encoder connections.

3. **Motor Direction Test (Test Number: 2):**
   - Verify motor direction.

4. **IMU Connections Test (Test Number: 3):**
   - Validate IMU (Inertial Measurement Unit) connections.

5. **Display Connections Test (Test Number: 4):**
   - Confirm proper connections with the display.

### How to Run Diagnostics

To run the diagnostic tests, follow these steps:

1. On your tortoisebotpromax terminal, launch the `bringup.launch` file:
   ```bash
   roslaunch tortoisebotpromax_firmware bringup.launch
   ```

2. On your slave PC or another terminal of your tortoisebotpromax, run the diagnostics test script:
   ```bash
   python3 diagnostics_test.py
   ```

3. The script will guide you through the instructions for each diagnostic test. Follow the on-screen instructions carefully.

### Important Notes
- It is crucial to execute the tests with caution and follow the on-screen instructions for each test to ensure accurate results.
- Ensure that the robot has sufficient space to move during the motor direction test (Test Number: 2).
- If any issues are identified during the tests, refer to the specific diagnostic output for guidance on addressing the problem.

By following these instructions, you can perform diagnostic tests on the tortoisebotpromax robot to identify and resolve any issues with its components.

## 8. Joystick Control Instructions
![autojoy](img/autojoyteleop.png)



