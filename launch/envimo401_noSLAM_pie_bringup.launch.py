from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
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
    mapper_cfg    = os.path.join(config_path, 'mapper_params_online_async_pie.yaml')
    realsense_cfg = os.path.join(config_path, 'realsense2_camera_pie.yaml')
    ublox_cfg = os.path.join(config_path, 'ublox_gps.yaml')
    foxglove_cfg =  os.path.join(config_path, 'foxglove_bridge.yaml')

  
    # 1. Define cmd_vel_relay node and store it in a variable
    cmd_vel_relay = Node(
        package='chassis_enable',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen'
    )
    ld.add_action(cmd_vel_relay)

    # 2. Define SmartCar node
    smartcar_node = Node(
        package='segwayrmp',
        executable='SmartCar',
        name='SmartCar',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_const')],
        parameters=[{'serial_full_name': 'rpserialport'}]
    )

    # 3. Delay SmartCar until cmd_vel_relay is started
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=cmd_vel_relay,
            on_start=[smartcar_node]
        )
    ))


    # 4. chassis_enable_client
    ld.add_action(Node(
        package='chassis_enable',
        executable='chassis_enable_client',
        name='chassis_enable_client',
        output='screen'
    ))


    # 5. robot_state_publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    ))

    # 6. Include RealSense launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ),
        launch_arguments={
            'config_file': realsense_cfg
         # , 'log_level': 'fatal'
        }.items()
    ))

    #  7. LDA 01 _ laserscan
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hls_lfcd_lds_driver'),
                'launch', 'hlds_laser.launch.py'
            )
         ),
         launch_arguments={
                'port': '/dev/lidar',
                'frame_id': 'laser_link'
             }.items()
    ))

    # # 8. SLAM Toolbox
    # ld.add_action(Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[mapper_cfg]
    # ))

    # 9. Ublox_gps
    ld.add_action(Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[ublox_cfg]
    ))
    
    # 10. foxglove
    ld.add_action(IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch', 'foxglove_bridge_launch.xml'
            )
        ),
         launch_arguments={
            'port': '8765',
            'address': '0.0.0.0',
            'min_qos_depth': '1',
            'max_qos_depth': '3',
            'send_buffer_limit': '1048576'
        }.items()
    ))
            
         
        # 11. Include Nav2 bringup launch
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('nav2_bringup'),
    #             'launch', 'navigation_launch.py'
    #         )
    #     ),
    #     launch_arguments={'params_file': nav2_cfg}.items()
    # ))

    # 12. Shutdown on any process exit
    ld.add_action(RegisterEventHandler(
        OnProcessExit(on_exit=[Shutdown()])
    ))

    return ld

