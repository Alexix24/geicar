### This file describes the steps to configure the Jetson Nano board, from the OS to the ROS installation

**An image of the SD card with all the necessary configurations and the initial ROS nodes is available here : .....**



## OS Installation
OS installation instructions for the Nvidia jetson board are available here: "https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit". Follow these instructions with :\
login = jetson\
hostname = geicar\
password = geicar

## General Updates and tools installation **(internet connection required)**

```sh
sudo apt update
sudo apt install nano
apt install net-tools
sudo apt install iputils-ping
sudo apt-get install ufw
```

## Firewall configuration :
1. Disable IPV6 in "/etc/default/ufw" : change IPV6=yes to IPV6=no 

2. Configure firewall rules :
```sh
sudo ufw allow from 192.168.1.1
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw disable
sudo ufw enable
```

3. Loopback multicast configuration :
```sh
route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
ifconfig lo multicast
```

4. Eth0 multicast configuration :
```sh
route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
ifconfig eth0 multicast
```

## Optionnal - Installation of ROS2 Dashing (if you want to use rviz2) **(internet connection required)**
The procedure to install ros2 dashing can be found here : "https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html"
**You need to install the "desktop" version : "ros-dashing-desktop"**

## Installation of ROS2 Humble with a docker container **(internet connection required)**

Source : "https://hub.docker.com/r/arm64v8/ros/tags"

1. Connect the LIDAR and the CAMERA to the Jetson board (USB)

2. Configure USB device :
```sh
sudo chmod 666 /dev/ttyUSB0 
```

3. Pull the ros-humble docker image :
```sh
sudo docker pull arm64v8/ros:humble-ros-base 
```

4. Create a ros-humble docker container :
```sh
sudo docker run -it --device=/dev/ttyUSB0 --device=/dev/video0 --net=host --name=ros-humble arm64v8/ros:humble-ros-base 
```

## LIDAR configuration **(internet connection required)**

1. Go into the docker container "ros-humble" :
```sh
docker start -ai ros-humble
```

2. Create a ROS2 workspace :
```sh
mkdir ~/ros2_ws/
mkdir ~/ros2_ws/src
```

3. Install the rplidar_ros2 package :\
Source : "http://wiki.ros.org/rplidar"
```sh
cd ~/ros2_ws/src
git clone https://github.com/babakhani/rplidar_ros2.git
cd ..
colcon build
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

4. You can check that the LIDAR is working with :
```sh
ros2 launch rplidar_ros rplidar.launch.py
```
If the installation was successful, the LIDAR should be running.

## CAMERA configuration **(internet connection required)**

1. Install v4l-utils :
```sh
sudo apt-get update 
sudo apt-get install v4l-utils
```

2. Go into the docker container ros-humble :
```sh
sudo docker start -ai ros-humble
```

3. Install v4l-utils :
```sh
sudo apt-get update 
sudo apt-get install v4l-utils
```

4. Install ros-humble-usb-cam package :\
Source : "https://github.com/ros-drivers/usb_cam/tree/ros2"
```sh
sudo apt-get install ros-humble-usb-cam 
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

5. You can check that the CAMERA is working with :
```sh
ros2 run usb_cam usb_cam_node_exe
```

If the installation was successful, the CAMERA should be ON.

## Source and configure the ROS environment

1. Go into the docker container ros-humble :
```sh
sudo docker start -ai ros-humble
```

2. Source the ROS environment :\
**Replace XX by the car number (1,2,3,4 ...). Example : "export ROS_DOMAIN_ID=1" for the car n°1**

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=XX" >> ~/.bashrc  
```
