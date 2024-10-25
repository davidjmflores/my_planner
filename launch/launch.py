from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # AprilTag Node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            output='screen',
            name='apriltag_node',
            parameters=[{
                'image_transport': 'raw',
                'camera_frame': 'base_link',
                'size': 0.16
            }],
            remappings=[
                ('/image_rect', '/Differential_drive_bot/camera_sensor/image_raw'),
                ('/camera_info', '/Differential_drive_bot/camera_sensor/camera_info')
            ]
        ),
        
        # Robot Controller with AprilTags Node
        Node(
            package='my_planner',
            executable='robot_controller', 
            output='screen',
            name='robot_controller'
        )
    ])
