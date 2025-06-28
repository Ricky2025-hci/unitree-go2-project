# ROS 2 Environment Setup for Unitree GO2

This repository document is the step-by-step procedure to set up a development environment for the Unitree GO2 quadruped robot using **ROS 2 Humble** on **Ubuntu 22.04**.

<p align="center">
  <img src="https://github.com/user-attachments/assets/ef567805-e2e3-41f6-a8b7-9764255cd352" width="45%">　<img src="https://github.com/user-attachments/assets/c8eca5bf-309e-4ca2-ba3d-7054b7563b4c" width="45%">
</p>

---

## System Requirements

- Ubuntu 22.04 (64-bit)
- ROS 2 Humble Hawksbill
- Internet connection
- Unitree GO2 robot
- Camera D435i
- LiDAR Heisai-XT-16

★ This setup and controlling Unitree GO2 was successfully tested using the following devices:
- Note PC (RYZEN 5)
- Mini PC (ODYSSEY BLUUE) 
- RaspberryPi (RaspberryPi 4)

Note: If you are planning to run Unitree GO2 with it's additional LiDAR or Camera, we recomend you to use Note PC or Mini PC for smooth conection.

---

## 1. Installing ROS 2 Humble

### Step 1: Update system

Update the package index and upgrade all installed packages to the latest version.

```bash
sudo apt update
sudo apt upgrade
```

### Step 2: Enable the universe repository

Adds the "universe" repository, which contains community-maintained packages needed for ROS. Then update the package index.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
```

### Step 3: Set the system locales to UTF-8

ROS 2 requires the system locale to be set to en_US.UTF-8 to avoid encoding issues during compilation and runtime.

```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 4: Add ROS 2 GPG key and source list

Add the ROS 2 repository key for verifying packages.

```bash
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Add the ROS2 package repository for your Ubuntu release.

```bash
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
```

### Step 5: Install ROS 2 Humble Desktop

Install the full desktop version of ROS 2 Humble, which includes rviz2, gazebo, rqt, and more.

```bash
sudo apt install ros-humble-desktop
```

Automatically sources the ROS 2 environment on terminal startup.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Clone the Unitree ROS 2 Repository

Clone the official Unitree ROS 2 packages and tools needed for controlling the GO2 robot. Then move to the folder "unitree_ros2"

```bash
git clone https://github.com/unitreerobotics/unitree_ros2
cd unitree_ros2
```

## 3. Install Requied ROS 2 Dependencies

These packages are required to enable DDS communication between ROS 2 and the Unitree robot using CycloneDDS middleware. Then install "gedit", the default text editor for the GNOME desktop environment.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
sudo apt install gedit
```

If not already installed:
Install the colcon build tool, used for building ROS 2 workspaces.

```bash
sudo apt install python3-colcon-common-extensions
```

## 4. Temporarily Disable Default ROS 2 Sourcing

This is done to avoid conflicts when using the setup.sh script from Unitree that manages the environment.

```bash
sudo gedit ~/.bashrc
```

Edit your .bashrc and comment out the following line:
```bash
# source /opt/ros/humble/setup.bash
```

## 5. Rebuild CycloneDDS from Source

Remove pre-existing versions of CycloneDDS.

```bash
cd ~/unitree_ros2/cyclonedds_ws/src
rm -rf rmw_cyclonedds cyclonedds
```

Clone the correct versions compatible with ROS 2 Humble.

```bash
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
```

Build only the CycloneDDS packages using colcon.

```bash
cd ..
colcon build --packages-select cyclonedds
```

If you get an error like so, go back to 3. Install Requied ROS 2 Dependencies.

```bash
ERROR: colcon not found
```

## 6. Build the Entire Workspace

Build the entire workspace including Unitree packages.

```bash
source /opt/ros/humble/setup.bash
colcon build
```

## 7. Configure Network Interface for GO2 Communication

First, connect Unitree GO2 and your computer using Ethernet cable. Then use the following to view the network interface that the robot connected.
Next, open the network settings and go to IPv4 settings. Then change the IPv4 mode to manual,and set the adress and the mask to the following.

```bash
Address: 192.168.123.99
Netmask: 255.255.255.0
```
Click apply and wait for the network to be connected. This allows the computer to be on the same subnet as the GO2 robot.

## 8. Edit setup.sh for Customization

Open setup.sh in unitree_ros2 folder by gedit. The setup.sh script sets the necessary environment variables for ROS 2 and network communication with the robot.

```bash
sudo gedit ~/unitree_ros2/setup.sh
```

First, change the ROS distribution from foxy or galactic to humble. Then modify the network interface to match your actual device (e.g. enp3s0, eth0).

## 9. Run and Test

List all available topics. If the robot is connected correctly, you will see Unitree-specific topics. There will be a lot. When this README file was first created, May 23rd 2025, length of the list was 104 lines.

```bash
/api/assistant_recorder/request
/api/assistant_recorder/response
/api/audiohub/request
/api/audiohub/response
/api/bashrunner/request
/api/bashrunner/response
/api/config/request
/api/config/response
...
```

---

## 10. Testing Example file

Before creating your own file to controll the Unitree GO2, use one of the example file provided. If the Unitree GO2 doesn't move as it should be, go back 1 to 9 above.
The following example demonstrates how to send a forward motion command to the Unitree GO2 robot using the ROS 2 `unitree_api::msg::Request` message.

|File name|Action|
|--|--|
|move_forward.cpp|Walk forward for 1m and stop.|
|move_rotate_stop.cpp|Walk and turn arround. Then repeat again and stop.|

### Step 1. Download example file

Download .cpp file from example folder to your local folder. The place to download is the following.

```bash
Home/unitree_ros2/example/src/src
```

Inside the src folder looks like the following after download. Be aware that we use "test_move.cpp" as the example file.

### Step 2. Edit CmakeLists.txt

Edit CmakeLists.text by opening it as follow.

```bash
gedit ~/unitree_ros2/example/src/CMakeLists.txt
```

Append the following lines to register your example .cpp file.
Make sure to change "test_move" to your own file name that you have downloaded or made in Step 1.
**Do not delete any line in the CmakeLists.txt.**

Tell Cmake to build an executable named test_move from your source files.

```bash
add_executable(test_move src/test_move.cpp src/common/ros2_sport_client.cpp)
```

Tell Cmake to link it with ROS 2 dependencies.

```bash
ament_target_dependencies(test_move ${DEPENDENCY_LIST})
```

Tell Cmake to install it to appropriate directory so it can be executed.

```bash
install(TARGETS 
        test_move
        DESTINATION lib/${PROJECT_NAME})
```

### Step 3. Build the Workspace

Build ROS 2 workspace.
Be sure that setup.sh has correctly set environment variables, especially for ROS 2 Humble.

```bash
source ~/unitree_ros2/setup.sh
cd ~/unitree_ros2/example
colcon build
```

The compiled binary will be available.

### Step 4. Run the Excutable

Run your build code.
Change "test_move" to the file name you have downloaded or created.

```bash
./install/unitree_ros2_example/lib/unitree_ros2_example/test_move
```

Your file may not be in the same directory.

