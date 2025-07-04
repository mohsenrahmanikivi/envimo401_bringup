from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = get_package_share_directory('envimo401')
    config_path = os.path.join(pkg_path, 'config')
    urdf_path = os.path.join(pkg_path, 'URDF', 'envimo_all_joints_fixed.urdf')

    # Define config file paths
    twist_mux_config = os.path.join(config_path, 'twist_mux.yaml')
    depthimage_config = os.path.join(config_path, 'depthimage_to_laserscan.yaml')
    nav2_config = os.path.join(config_path, 'nav2_params_test.yaml')
    mapper_config = os.path.join(config_path, 'mapper_params_online_async.yaml')
    realsense_config = os.path.join(config_path, 'realsense2_camera.yaml')

    # 1. twist_mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config]
    )

    # 2. Segway RMP - SmartCar
    segway_node = Node(
        package='segwayrmp',
        executable='SmartCar',
        name='segway_driver',
        remappings=[
            ('/cmd_vel', '/cmd_vel_out')
        ],
        output='screen',
        prefix='xterm -hold -e'  # <-- open an xterm and keep it open after exit
    )

    # 3. Segway RMP - drive_segway_sample
    segway_drive_node = Node(
        package='segwayrmp',
        executable='drive_segway_sample',
        name='segway_driver_sample',
        remappings=[
            ('/cmd_vel', '/cmd_vel_drive')
        ]
    )

    # 4. Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_path).read()
        }]
    )

    # 5. RealSense Camera launch
    realsense_node = Node(
        package='realsense2_camera',
        executable='rs_launch.py',
        name='realsense_camera',
        parameters=[{'config_file': realsense_config}]
    )

    # 6. DepthImage to LaserScan
    depthimage_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[depthimage_config],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info')
        ]
    )

    # 7. SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[mapper_config]
    )

    # 8. Nav2 Bringup
    nav2_node = Node(
        package='nav2_bringup',
        executable='navigation_launch.py',
        name='nav2_bringup',
        parameters=[nav2_config]
    )

    # Add all nodes to launch description
    ld.add_action(twist_mux_node)
    ld.add_action(segway_node)
    ld.add_action(segway_drive_node)
    ld.add_action(robot_state_pub_node)
    ld.add_action(realsense_node)
    ld.add_action(depthimage_node)
    ld.add_action(slam_node)
    ld.add_action(nav2_node)

    return ld
