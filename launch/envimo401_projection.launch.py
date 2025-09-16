import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory





def generate_launch_description():
    # --- locate your own package ---
    camera_name     = 'camera'  # Default value
    
    pkg_path        = get_package_share_directory('envimo401_bringup')
    config_dir      = os.path.join(pkg_path, 'config')
    projections_yaml = os.path.join(config_dir, 'projections.yaml')
    camera_yaml      = os.path.join(config_dir, 'camera_cameras.yaml')

    return LaunchDescription([

        # --- Front view pinhole projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name='camera_pinhole_front',
            parameters=[projections_yaml, camera_yaml],
            remappings=[
                ('~/projection', 'camera/pinhole_front/image_rect_color'),
                ('~/camera_info', 'camera/pinhole_front/camera_info')
            ],
            output='screen'
        ),

        # --- Mercator projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name='camera_mercator_projection',
            parameters=[projections_yaml, camera_yaml],
            remappings=[
                ('~/projection', 'camera/mercator')
            ],
            output='screen'
        ),

        # --- Left fisheye projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name='camera_left_fisheye',
            parameters=[projections_yaml, camera_yaml],
            remappings=[
                ('~/projection', 'camera/left/ideal_fisheye')
            ],
            output='screen'
        ),

        # --- Right fisheye projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name='camera_right_fisheye',
            parameters=[projections_yaml, camera_yaml],
            remappings=[
                ('~/projection', 'camera/right/ideal_fisheye')
            ],
            output='screen'
        ),
    ])
