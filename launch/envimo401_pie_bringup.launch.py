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
    urdf_path     = os.path.join(pkg_path, 'urdf', 'envimo_all_joints_fixed_with_laser.urdf')


    # Config files
    nav2_cfg      = os.path.join(config_path, 'nav2_params_with_slam.yaml')
    mapper_cfg    = os.path.join(config_path, 'mapper_params_online_async.yaml')
    realsense_cfg = os.path.join(config_path, 'realsense2_camera_pie.yaml')
    leaser_cfg    = os.path.join(config_path, 'hls_lfcd_lds_driver_pie.yaml')
  

    # 1. cmd_vel_relay
    ld.add_action(Node(
        package='chassis_enable',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen'
    ))

    # 2. SegwayRMP SmartCar (inherits LD_LIBRARY_PATH)
    ld.add_action(Node(
        package='segwayrmp',
        executable='SmartCar',
        name='SmartCar',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_const')]
    ))


    # 3. chassis_enable_client
    ld.add_action(Node(
        package='chassis_enable',
        executable='chassis_enable_client',
        name='chassis_enable_client',
        output='screen'
    ))


    # 4. robot_state_publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    ))

    # # 5. Include RealSense launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ),
        launch_arguments={'config_file': realsense_cfg}.items()
    ))

    #  6. LDA 01 _ laserscan
    
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hls_lfcd_lds_driver'),
                'launch', 'hlds_laser.launch.py'
            )
         ),
         launch_arguments={'params_file': leaser_cfg}.items()
    ))

    # 7. SLAM Toolbox
    # ld.add_action(Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[mapper_cfg]
    # ))

    # 8. Include Nav2 bringup launch
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('nav2_bringup'),
    #             'launch', 'navigation_launch.py'
    #         )
    #     ),
    #     launch_arguments={'params_file': nav2_cfg}.items()
    # ))

    # 9. Shutdown on any process exit
    ld.add_action(RegisterEventHandler(
        OnProcessExit(on_exit=[Shutdown()])
    ))

    return ld
