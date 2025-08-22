# Envimo401 on PI5
This is the bring-up package including all instructions needed to run the envimo401 

Requirments:
- Raspberry Pi 5
- Ubuntu 24.04
- ros2 jazzy
# 1. Install ROS2 Jazzy
1. Follow the instructions https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
2. Make workspace folder "~/ros2_ws/src" 
3. add source command to the "~/.bashrc"
```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```
# 2. Install the RealSense driver package for ROS2
Installing the packages reference https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu

A. Install latest Intel® RealSense™ SDK 2.0:  
 ```
sudo apt install ros-$ROS_DISTRO-librealsense2*
```

B. Install ROS Wrapper for Intel® RealSense™ cameras  

```
sudo apt install ros-$ROS_DISTRO-realsense2-*
```

C. Set UDEV rules by downloading the official rules into place
```
sudo curl -fsSL \
  https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
  -o /etc/udev/rules.d/99-realsense-libusb.rules
```
set permission and reload
```
sudo chmod 644 /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```


  ### Optional for test
Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.


# 3. Install Lds-01 driver package for ROS2
A. Installation:  
 ```
sudo apt install ros-$ROS_DISTRO-hls-lfcd-lds-driver
```
  
B. Set UDEV rules to have a persistence "\dev\lidar" instead ot "\dev\ttyUSBX" 

i. Identify Your Device tty
```sudo dmesg | grep tty```

ii. Identify Your Device if you know which tty is:
 ```udevadm info -a -n /dev/ttyUSB0 | grep "KERNELS=="```
 
iii. Create a udev rule to have a persistent name “\dev\lidar”:
```sudo nano /etc/udev/rules.d/99-lidar.rules```

iv. Put this line and modify the KERNELS and SYNKINk
```SUBSYSTEM=="tty", KERNELS=="2-1.1", SYMLINK+="lidar", MODE="0777", GROUP="dialout"```

v. Add your user to dialout group
```sudo usermod -aG dialout $USER```

vi. reload
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

vii. test
```
ls /dev
cat /dev/lidar
```

C. Run hlds_laser_publisher Node to test:  
```
ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py
```

# 4. Ublox driver
A.Install 
```
sudo apt install ros-$ROS_DISTRO-ublox-gps
```
B. Test
```
source /opt/ros/humble/setup.bash
ros2 run ublox_gps ublox_gps_node --ros-args -p device:=/dev/ttyACM0
```

# 5. Install foxglove-bridge

```
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

# 6. Install slam toolbox
```
sudo apt install ros-$ROS_DISTRO-slam-toolbox

```
# 7. Install nav2 stack 

```
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
```

# 8. "chassis" and "bringup" packages
Clone both of them by going to the src folder of the workspace and cloning the necessary packages
A. Bringup package :  
```
cd ~/ros2_ws/src && \
git clone https://github.com/mohsenrahmanikivi/envimo401_bringup.git
```

B. Chassis package:  
 ```
cd ~/ros2_ws/src && \
git clone https://github.com/mohsenrahmanikivi/envimo401_chassis.git
```

C. Add the library path to the "~/.bashrc"
 ```
# Prepend path to LD_LIBRARY_PATH if not already included
if [[ ":$LD_LIBRARY_PATH:" != *":$HOME/ros2_ws/src/envimo401_chassis/LibAPI/lib:"* ]]; then
    export LD_LIBRARY_PATH="$HOME/ros2_ws/src/envimo401_chassis/LibAPI/lib:$LD_LIBRARY_PATH"
fi
 ```
D. Set UDEV rules to have a persistence "\dev\rpserialport" instead ot "\dev\ttyUSBX" 

0i. Create a bash file to set the speed ```sudo nano /usr/local/bin/set_rpserial_baud.sh```
```
#!/bin/bash
stty -F /dev/rpserialport 921600
```
0ii. set permission ```sudo chmod +x /usr/local/bin/set_rpserial_baud.sh```
  
i. Identify Your Device tty
```sudo dmesg | grep tty```

ii. Identify Your Device if you know which tty is:
 ```udevadm info -a -n /dev/ttyUSB1 | grep "KERNELS=="```
 
iii. Create a udev rule to have a persistent name “\dev\rpserialport”:
```sudo nano /etc/udev/rules.d/99-rpserialport.rules```

iv. Put this line and modify the KERNELS and SYNKINk
```
SUBSYSTEM=="tty", KERNELS=="4-2", SYMLINK+="rpserialport", MODE="0777", RUN+="/usr/local/bin/set_rpserial_baud.sh"
```

v. reload
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

vi. test
```
ls /dev
cat /dev/rpserialport
```

# 9. Build the packages    
`colcon build`

 
# 10. Run   
```
source ~/.bashrc
ros2 launch envimo401_bringup envimo401_bringup.launch.py
```
 



