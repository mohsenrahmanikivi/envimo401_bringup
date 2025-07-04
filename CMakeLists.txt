cmake_minimum_required(VERSION 3.10)
project(envimo401)

# Find dependencies
find_package(ament_cmake REQUIRED)


# Install launch and config files for easy reference
install(
  DIRECTORY launch config URDF
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
