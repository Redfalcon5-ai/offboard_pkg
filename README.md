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

This installation requires a cmake version >= 3.20 and it is probably the case that you have 3.13.3 or similar. At the time of writing this, cmake 4.0 has just been released, but please use cmake 3.20 to 3.31.5. Follow these steps to upgrade it:

1) [Download](https://cmake.org/download/) the latest cmake bash script. In my case, it was ```cmake-3.31.5-Linux-x86_64.sh```
2) Move the script to your home directory and copy it to /opt/ using ```sudo cp ~/cmake-3.*version*-Linux-x86_64.sh /opt/```
3) Give the script executable permissions by running ```sudo chmod +x /opt/cmake-3.*version*.sh```
4) ```cd /opt/``` and run the script: ```sudo bash /opt/cmake-3.*version*.sh```. You will have to answer ```y``` twice
5) Run ```sudo ln -s /opt/cmake-3.*version*/bin/* /usr/local/bin```
6) Check the version: ```cmake --version```
7) You will have to run the PX4 setup script again. Navigate to your home directory and run ```bash ./PX4-Autopilot/Tools/setup/ubuntu.sh```

Once cmake is updated, follow these steps:
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

Create a ROS package for your own code (OR skip this step and just clone this repo as you did with PX4_msgs above). If you make your own package, copy this [code](https://github.com/Redfalcon5-ai/offboard_pkg/blob/main/offboard_pkg/offboard_node.py) into ```offboard_pkg/offboard_node.py``` after making the package.
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
