# Description

This is a ROS 2 offboard control package that contains a node for takeoff and landing 

# Installation

If you have ROS 1 installed, you will face headaches when making a workspace, so get rid of it!

## PX4 Install

Install PX4: [https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)

## ROS 2 Install

Install ROS 2 Desktop + ROS Development Tools: [https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

Add ```source /opt/ros/foxy/setup.bash``` to your .bashrc and run ```source ~/.bashrc```

## Micro-XRCE-DDS-Agent Install

In your home directory:
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/ ```
```

## Workspace Setup

Create a directory for the workspace:
```
mkdir -p ~/ws_offboard/src/
cd ~/ws_offboard/src/
```

Clone PX4_msgs & PX4_ros_com:
```
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

Create a ROS package for your own code (OR skip this step and just clone this repo as you did with PX4_msgs above):
```
ros2 pkg create --build-type ament_python --node-name offboard_node offboard_pkg
```

Build the workspace with symlink so you won't have to rebuild after every edit: 
```
cd ~/ws_sensor_combined/
colcon build --symlink-install
```

Add ```source ~/ws_offboard/install/local_setup.bash``` to your .bashrc and run ```source ~/.bashrc```

## Adding Files
In the future, if you want to add more nodes, you can just add new python files in the nodes directory (```ws/src/pkg/pkg/```). You also have to add new entry points in setup.py:
```
entry_points={
        'console_scripts': [
            'offboard_node = offboard_pkg.offboard_node:main',
            <------------------ Add Here ------------------>
        ],
    },
```

You will also have to rebuild the workspace: 
```
cd ~/ws_sensor_combined/
colcon build --symlink-install
```

# Running Code

Open 3 Terminals
- 1 - PX4: In the PX4 directory, run ```make px4_sitl gazebo-classic```
- 2 - XRCE_DDS: In the Micro-XRCE-DDS-Agent directory, run ```MicroXRCEAgent udp4 -p 8888```
- 3 - ROS node: In the workspace, run ```ros2 run offboard_pkg offboard_node```
