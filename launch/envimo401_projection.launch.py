from launch import LaunchDescription
from launch_ros.actions import Node
import os



# --- locate your own package ---
camera_name = "camera"  # Default value

pkg_path         = get_package_share_directory('envimo401_bringup')
config_dir      = os.path.join(pkg_path, 'config')

projections_yaml = os.path.join(config_dir, 'projections.yaml')
camera_yaml = os.path.join(config_dir, f'{camera_name}_cameras.yaml')

def generate_launch_description():

    return LaunchDescription([

        # --- Front view pinhole projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name=f'{camera_name}_pinhole_front',
            parameters=[projections_yaml, camera_yaml],
            remappings=[
                ('~/projection', f'/{camera_name}/pinhole_front/image_rect_color'),
                ('~/camera_info', f'/{camera_name}/pinhole_front/camera_info')
            ],
            output='screen'
        ),

        # --- Mercator projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name=f'{camera_name}_mercator_projection',
            parameters=[projections_yaml, camera_yaml],
            remappings=[
                ('~/projection', f'/{camera_name}/mercator')
            ],
            output='screen'
        ),

        # --- Left fisheye projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name=f'{camera_name}_left_fisheye',
            parameters=[projections_yaml, camera_yaml],
            remappings=[
                ('~/projection', f'/{camera_name}/left/ideal_fisheye')
            ],
            output='screen'
        ),

        # --- Right fisheye projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name=f'{camera_name}_right_fisheye',
            parameters=[projections_yaml, camera_yaml],
            remappings=[
                ('~/projection', f'/{camera_name}/right/ideal_fisheye')
            ],
            output='screen'
        ),
    ])
