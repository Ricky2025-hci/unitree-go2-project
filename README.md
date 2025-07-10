# Notion!
This repository is open for temporary. After 31st August 2025, this repository will be changed to **Private** until this project is finished.


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
- Camera D435i (If needed)
- LiDAR Heisai-XT-16 (If needed)

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

<p align="center">
  <img src="https://github.com/user-attachments/assets/e837d3db-17de-42eb-bf0c-b86edb621e6b" width="50%">
</p>


## 8. Edit setup.sh for Customization

Open setup.sh in unitree_ros2 folder by gedit. The setup.sh script sets the necessary environment variables for ROS 2 and network communication with the robot.

```bash
sudo gedit ~/unitree_ros2/setup.sh
```

First, change the ROS distribution from foxy or galactic to humble. Then modify the network interface to match your actual device (e.g. enp3s0, eth0).

<p align="center">
  <img src="https://github.com/user-attachments/assets/3b7083c5-2f63-4398-95c0-e5e16994ea1e" width="80%">
</p>

## 9. Run and Test

List all available topics. If the robot is connected correctly, you will see Unitree-specific topics. 

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```

There will be a lot of topics like below. When this README file was first created, May 23rd 2025, length of the list was 104 lines.

<p align="center">
  <img src="https://github.com/user-attachments/assets/c42fff6d-1ead-4916-98be-8088d1f785d3" width="50%">
</p>

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

Inside the src folder looks like the following picture after download. Be aware that we use "test_move.cpp" as the example file.

<p align="center">
  <img src="https://github.com/user-attachments/assets/ab94b59a-44e6-40f9-8ff4-87158ee84aa3" width="50%">
</p>


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

---

# LiDAR Integration

This section explains how to set up and visualize the **Hesai XT16 LiDAR** sensor that connects to the Unitree GO2 robot via ROS 2 Humble.
Please finish **ROS 2 Environment Setup for Unitree GO2** written above.

**Note:** There have been some changes in the repository we use from Hesai Technology and we believe some of the files are missing. So, we have uploaded the raw file `humble_ws.zip`. (date:July 5th 2025)


## Step 1. Verify LiDAR Connectivity

Ensure the LiDAR device is physically connected and on the same subnet (`192.168.123.XXX`).

Configure IP adress in detail as follow.
First, ensure the LiDAR is conected and enter "192.168.123.20" in your web browser.
You should see the LiDAR configration page.
If you want to receive point cloud data via the Docking Station "192.168.123.18", set the Destination IP to 192.168.123.18.
If you want to receive data from an external PC instead, set the Destination IP to the IP address of that PC.
Alternatively, you can set the Destination IP to 255.255.255.255 as shown in the image, which allows any device within the 192.168.123.xxx subnet to receive the data.

If the LiDAR has connected successfully, you should receive responses from DockingStation by running the following.

```bash
ping 192.168.123.20
```

If not, check your network settings or powersupply.

## Step 2. Clone the Driver Repository

Create a new ROS 2 workspace, if you haven't already done so.
The "--recurse-submodules" flag is important to include required libraries.

```bash
mkdir -p ~/humble_ws/src
cd ~/humble_ws/src
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
```

## Step 3. Edit the Configuration File

Edit the LiDAR configuration to match the IP address by opening "config.yaml" as follow.

```bash
sudo gedit ~/humble_ws/src/HesaiLidar_ROS_2.0/config/config.yaml
```

Update the following line. Make sure the IP matches your LiDAR device's static IP.

```bash
device_ip_address: 192.168.123.20
```

## Step 4. Build the Drive

Build the Hasai LiDAR driver and links packages for development.

```bash
cd ~/humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

If you have encountered some CMake Error like picture bellow, you are likely to get an error at the end of "Step 5. Terminal 1".

<p align="center">
  <img src="https://github.com/user-attachments/assets/ad50a1c5-dd6a-4b3a-9a15-29ff95ae2c33" width="75%">
</p>


## Step 5. Run the Driver

Open two terminals and find the pass you need.

```bash
find ~/unitree_ros2 -name setup.sh
```

### Terminal 1 : Launch the LiDAR node

Source the Unitree environment and the LiDAR workspace.

```bash
source ~/unitree_ros2/setup.sh
source install/setup.bash
```

Then run the driver.

```bash
ros2 run hesai_ros_driver hesai_ros_driver_node
```

If you have encountered some error like picture bellow, try **After running `colcon build` and trying to launch a node with `ros2 run` but have "OSError: [Errno 8] Exec format error:"** written bellow.

<p align="center">
  <img src="https://github.com/user-attachments/assets/52041eb6-f197-41f7-a526-96075018d934" width="75%">
</p>


### Terminal 2 : Visualize using RViz2

Source the Unitree environment and the LiDAR workspace.

```bash
source ~/unitree_ros2/setup.sh
source install/setup.bash
```

Then run RViz2.

```bash
rviz2
```

In RViz, change the followings.

 - Fixed Frame: hesai_lidar
 - Ropic: /lidar_points
 - Add Display type: PointClound2

See the LiDAR point cloud rendered in real time to check the fixd frame.

1

If you see something simiar, you have successfully conected LiDAR.

<p align="center">
  <img src="https://github.com/user-attachments/assets/544c03ca-016b-4b58-89e8-77f7e526269c" width="100%">
</p>


### After running `colcon build` and trying to launch a node with `ros2 run` but have "OSError: [Errno 8] Exec format error:"

This typically occurs when the build failed partially and left behind an incomplete or corrupted executable.

Clean the build artifacts and rebuild from scratch:

```bash
cd ~/your_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

If there are no error, run the "Terminal 2".
