from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    ld = LaunchDescription()

    # --- locate your own package ---
    pkg_path      = get_package_share_directory('envimo401_bringup')
    config_path   = os.path.join(pkg_path, 'config')
    urdf_path     = os.path.join(pkg_path, 'urdf', 'envimo_all_joints_fixed.urdf')


    # Config files
    twist_mux_cfg = os.path.join(config_path, 'twist_mux.yaml')
    depthimg_cfg  = os.path.join(config_path, 'depthimage_to_laserscan.yaml')
    nav2_cfg      = os.path.join(config_path, 'nav2_params_test.yaml')
    mapper_cfg    = os.path.join(config_path, 'mapper_params_online_async.yaml')
    realsense_cfg = os.path.join(config_path, 'realsense2_camera.yaml')

    # 1. twist_mux
    ld.add_action(Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_cfg]
    ))

    # 2. SegwayRMP SmartCar (inherits LD_LIBRARY_PATH)
    ld.add_action(Node(
        package='segwayrmp',
        executable='SmartCar',
        name='segway_driver',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_out')]
    ))

    # # 3. drive_segway_sample in its own terminal
    # ld.add_action(ExecuteProcess(
    #     cmd=[
    #         'gnome-terminal', '--',
    #         'bash', '-c',
    #         'ros2 run segwayrmp drive_segway_sample --ros-args --remap /cmd_vel:=/cmd_vel_drive; exec bash'
    #     ],
    #     output='screen'
    # ))

    # 4. robot_state_publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    ))

    # 5. Include RealSense launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ),
        launch_arguments={'config_file': realsense_cfg}.items()
    ))

    # 6. depthimage_to_laserscan
    ld.add_action(Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[depthimg_cfg],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info')
        ]
    ))

    # 7. SLAM Toolbox
    ld.add_action(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapper_cfg]
    ))

    # 8. Include Nav2 bringup launch
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={'params_file': nav2_cfg}.items()
    ))

    # 9. Shutdown on any process exit
    ld.add_action(RegisterEventHandler(
        OnProcessExit(on_exit=[Shutdown()])
    ))

    return ld
