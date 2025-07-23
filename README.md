# 1. Install ros2
# 2. Install RealSense driver package for ros2
 ref  https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu
## Installing the packages:
- Register the server's public key:
```
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

- Make sure apt HTTPS support is installed:
`sudo apt install apt-transport-https`

- Add the server to the list of repositories:
```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
```

- Install the libraries (see section below if upgrading packages):  
  `sudo apt install librealsense2-dkms`  
  `sudo apt install librealsense2-utils`  
  The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.  

- Optionally install the developer and debug packages:  
  `sudo apt install librealsense2-dev`  
  `sudo apt install librealsense2-dbg`  
  With `dev` package installed, you can compile an application with **librealsense** using `g++ -std=c++11 filename.cpp -lrealsense2` or an IDE of your choice.

Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.

# 4. Install Lds-01 driver
- Installation 
  `sudo apt install ros-humble-hls-lfcd-lds-driver`
- Set Permission for LDS-01
  `sudo apt install ros-humble-hls-lfcd-lds-driver`
- Run hlds_laser_publisher Node to test
  `ros launch hls_lfcd_lds_driver hlds_laser.launch`
- Run hlds_laser_publisher Node with RViz
  `ros launch hls_lfcd_lds_driver view_hlds_laser.launch`
  
# 5. clone both chasses and bringup pakages
# 6. build them

# envimo infrastructure
## 1. URDF file 
## 2. Chassis driver
envimo401_chassis

## 3. GPS driver
envimo401_gps
https://github.com/mohsenrahmanikivi/ros2_driver_ublox_gps_module
## 4. Camera driver


