import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


camera_info_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

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
            parameters=[projections_yaml, 
                        camera_yaml,
                       {"use_intra_process_comms": True}
                       ],
                        
            remappings=[
                ('/projection', 'camera/pinhole_front/image_rect_color'),
                ('/camera_info', 'camera/pinhole_front/camera_info')
            ],
            output='screen',
            qos_profile=camera_info_qos
        ),

        # --- Mercator projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name='camera_mercator_projection',
            parameters=[projections_yaml, 
                        camera_yaml,
                        {"use_intra_process_comms": True}
                       ],
            remappings=[
                ('/projection', 'camera/mercator')
            ],
            output='screen',
            qos_profile=camera_info_qos
        ),

        # --- Left fisheye projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name='camera_left_fisheye',
            parameters=[projections_yaml, 
                        camera_yaml,
                       {"use_intra_process_comms": True}
                       ],
            remappings=[
                ('/projection', 'camera/left/ideal_fisheye')
            ],
            output='screen',
            qos_profile=camera_info_qos
        ),

        # --- Right fisheye projection ---
        Node(
            package='image_projection',
            executable='periodic_image_projection_standalone_node',
            name='camera_right_fisheye',
            parameters=[projections_yaml, 
                        camera_yaml,
                        {"use_intra_process_comms": True}
                       ],
            remappings=[
                ('/projection', 'camera/right/ideal_fisheye')
            ],
            output='screen',
            qos_profile=camera_info_qos
        ),
    ])
