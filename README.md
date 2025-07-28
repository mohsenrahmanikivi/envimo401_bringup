# 1. Install ROS2 humble
# 2. Install RealSense driver package for ROS2
```
https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu
```
## Installing the packages:
1. Register the server's public key:
```
sudo mkdir -p /etc/apt/keyrings \
&& curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

2. Make sure apt HTTPS support is installed:
```
sudo apt install apt-transport-https
```
4. Add the server to the list of repositories:
```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list \
&& sudo apt update
```

4. Install latest Intel® RealSense™ SDK 2.0:  
  ```
sudo apt install ros-humble-librealsense2*
```

6. Install ROS Wrapper for Intel® RealSense™ cameras  
 ```
sudo apt install ros-humble-realsense2-*
```

  ### Optional for test
Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.


# 4. Install Lds-01 driver package for ROS2
1. Installation:  
  ```
sudo apt install ros-humble-hls-lfcd-lds-driver
```
  
3. Set Permission for LDS-01(change the port based on your desinge /dev/ttyUSB0):  
  ```
sudo chmod a+rw /dev/ttyUSB0
```
3.1 OR to set name /dev/lidar and permission create a udev rule 
Create a udev rule to have a persistent name “\dev\lidar”:
 ```sudo nano /etc/udev/rules.d/99-lidar.rules```
 and copy it there ,then reboot
```SUBSYSTEM=="tty", KERNELS=="1-1.3", SYMLINK+="lidar", MODE="0666", GROUP="dialout"```


sudo usermod -aG dialout $USER

- Run hlds_laser_publisher Node to test:  
```
ros2 launch hls_lfcd_lds_driver hlds_laser.launch.py
```
  
- Run hlds_laser_publisher Node with RViz:  
```
ros2 launch hls_lfcd_lds_driver view_hlds_laser.launch.py
```

# 5. Install nav2 stack

```
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

# 5. Install twist mux ( is about to be removed)
```
sudo apt install ros-humble-twist-mux
```

# 6. Clone both main packages
Go to the src folder and clone the necessary packages
1. Bringup package :  
```
git clone https://github.com/mohsenrahmanikivi/envimo401_bringup.git
```


3. Chassis package:  
 ```
git clone https://github.com/mohsenrahmanikivi/envimo401_chassis.git
```

5. Add the library to the library path:  
 `export LD_LIBRARY_PATH=<PATH_TO_THE_PACKAGE>/src/envimo401_chassis/LibAPI/lib:$LD_LIBRARY_PATH`  
 `export LD_LIBRARY_PATH=~/ros2_ws/src/envimo401_chassis/LibAPI/lib:$LD_LIBRARY_PATH`

5. Prepare the chassis:
 ```
cd ~/ros2_ws/src/envimo401_chassis
chmod +x Segway_RMP_Init.sh
sudo ./Segway_RMP_Init.sh
```


# 7. Build the packages    

 `colcon build`

 
 # 8. Run   
 
 `ros2 launch envimo401_bringup envimo401_bringup.launch.py`
 

# envimo infrastructure
## 1. URDF file
## 3. GPS driver
envimo401_gps
https://github.com/mohsenrahmanikivi/ros2_driver_ublox_gps_module



