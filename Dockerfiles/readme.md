## ros1_bridge ROS1/noetic ROS2/humble on l4t-jetpack:r36.4.0
- reference: https://forums.developer.nvidia.com/t/dockerfile-ros1-bridge-ros1-noetic-ros2-humble-on-l4t-jetpack-r36-4-0/331453
- Docker hub link : <code>docker pull pesare1404/ros1_ros2_bridge:latest  </code>


## run the Image
If your working space is not the same as "/ssd/workspaces", change it as it is going to be a shared folder witha  container
```bash
docker run -it --rm \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  -v "/ssd/workspaces:/root" \
  --privileged \
  --net=host \
  --name ros1_bridge \
  ros1_bridge:ros2
```

