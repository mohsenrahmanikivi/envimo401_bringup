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
    calib_path    = os.path.join(pkg_path, 'calibration')
    overlay_path    = os.path.join(pkg_path, 'overlay')
    


    # Config files
    urdf_path     = os.path.join(pkg_path, 'urdf', 'envimo_all_joints_fixed_with_laser.urdf')
   
    nav2_cfg      = os.path.join(config_path, 'nav2_params_with_slam.yaml')
    mapper_cfg    = os.path.join(config_path, 'mapper_params_online_async_pie.yaml')
    realsense_cfg = os.path.join(config_path, 'realsense2_camera_pie.yaml')
    ublox_cfg     = os.path.join(config_path, 'ublox_m8n.yaml')
    foxglove_cfg  = os.path.join(config_path, 'foxglove_bridge.yaml')
    hlds_laser_publisher_cfg= os.path.join(config_path, 'hlds_laser_publisher_pie.yaml')

    camera_left_calib        = os.path.join(calib_path, 'left', 'ost.yaml')
    camera_right_calib       = os.path.join(calib_path, 'right', 'ost.yaml')
    
    camera_left_overlay     = os.path.join(overlay_path, 'left_640_480.png')
    camera_right_overlay    = os.path.join(overlay_path, 'right_640_480.png')
    camera_center_overlay   = os.path.join(overlay_path, 'center_640_480.png')

  
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
            'config_file': realsense_cfg,
            'camera_name': 'center'
            
        }.items()
    ))


    #     #  8. camera_left_link  (5011)
    ld.add_action(Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_left',
            parameters=[{
                'gscam_config': f'udpsrc port=5011 ! jpegparse ! jpegdec ! videoconvert ! videoflip method=clockwise ! \
                queue ! gdkpixbufoverlay location={camera_left_overlay} ! queue ! videorate ! video/x-raw,format=BGR,framerate=15/1',
                'camera_name': 'left',
                'frame_id': 'camera_left_link',
                'camera_info_url': f'file://{camera_left_calib}'
            }],
            remappings=[
                ('/camera/camera_info', '/camera/left/camera_info'),
                ('/camera/image_raw', '/camera/left/image_raw'),
                ('/camera/image_raw/compressed', '/camera/left/image_raw/compressed'),
                ('/camera/image_raw/compressedDepth', '/camera/left/image_raw/compressedDepth'),
                ('/camera/image_raw/theora', '/camera/left/image_raw/theora'),
                ('/camera/image_raw/zstd', '/camera/left/image_raw/zstd')
            ]
        ))

    #     #  9. camera_right_link (5012)
    ld.add_action(Node(
            package='gscam',
            executable='gscam_node',
            name='gscam_right',
            parameters=[{
                'gscam_config': 'udpsrc port=5012 ! jpegparse ! jpegdec ! videoconvert ! videoflip method=counterclockwise ! video/x-raw,format=BGR',
                'camera_name': 'right',
                'frame_id': 'camera_right_link',
                'camera_info_url': f'file://{camera_right_calib}'
            }],
            remappings=[
                ('/camera/camera_info', '/camera/right/camera_info'),
                ('/camera/image_raw', '/camera/right/image_raw'),
                ('/camera/image_raw/compressed', '/camera/right/image_raw/compressed'),
                ('/camera/image_raw/compressedDepth', '/camera/right/image_raw/compressedDepth'),
                ('/camera/image_raw/theora', '/camera/right/image_raw/theora'),
                ('/camera/image_raw/zstd', '/camera/right/image_raw/zstd')
            ]
        ))

    #  7. Extra compressed image by image_transport
    ld.add_action(Node(
        package='image_transport',
        executable='republish',
        name='depth_republisher',
        output='screen',
        parameters=[
            {'in_transport': 'raw'},
            {'out_transport': 'compressed'},
            {'out.compressed.jpeg_quality': 15}
        ],
        remappings=[
            ('in', '/camera/center/color/image_raw'),
            ('out/compressed', '/camera/center/color/image_raw/extra_compressed')
        ]
    ))
    
    #  7. LDA 01 _ laserscan
    ld.add_action(Node(
        package='hls_lfcd_lds_driver',
        executable='hlds_laser_publisher',
        name='hlds_laser_publisher',
        output='screen',
        parameters=[hlds_laser_publisher_cfg]
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
        remappings=[('/ublox_gps_node/fix', '/fix'),
                   ('/ublox_gps_node/fix_velocity', '/fix_velocity')],
        parameters=[ublox_cfg]
    ))

    # 10. foxglove

    ld.add_action(Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[foxglove_cfg]
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

