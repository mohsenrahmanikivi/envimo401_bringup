from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = get_package_share_directory('envimo401')
    config_path = os.path.join(pkg_path, 'config')
    urdf_path = os.path.join(pkg_path, 'URDF', 'envimo_all_joints_fixed.urdf')

    # Config file paths
    twist_mux_config     = os.path.join(config_path, 'twist_mux.yaml')
    depthimage_config    = os.path.join(config_path, 'depthimage_to_laserscan.yaml')
    nav2_config          = os.path.join(config_path, 'nav2_params_test.yaml')
    mapper_config        = os.path.join(config_path, 'mapper_params_online_async.yaml')
    realsense_config     = os.path.join(config_path, 'realsense2_camera.yaml')

    # 1. twist_mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config]
    )
    ld.add_action(twist_mux_node)

    # 2. Segway RMP - SmartCar
    segway_node = Node(
        package='segwayrmp',
        executable='SmartCar',
        name='segway_driver',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_out')]
    )
    ld.add_action(segway_node)

    # 3. Segway RMP - drive_segway_sample in its own terminal
    segway_drive_node = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'bash', '-c',
            'ros2 run segwayrmp drive_segway_sample --ros-args --remap /cmd_vel:=/cmd_vel_drive; exec bash'
        ],
        output='screen'
    )
    ld.add_action(segway_drive_node)

    # 4. Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read()
        }]
    )
    ld.add_action(robot_state_pub_node)

    # 5. RealSense Camera
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ),
        launch_arguments={'config_file': realsense_cfg}.items()
    )
    
    ld.add_action(realsense_node)

    # 6. DepthImage to LaserScan
    depthimage_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[depthimage_config],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info')
        ]
    )
    ld.add_action(depthimage_node)

    # 7. SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapper_config]
    )
    ld.add_action(slam_node)

    # 8. Nav2 Bringup
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={'params_file': nav2_cfg}.items()
    )
    ld.add_action(nav2_node)

    return ld
