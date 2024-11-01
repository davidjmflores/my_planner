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
            executable='motion_controller_node', 
            output='screen',
            name='motion_controller_node',
            remappings=[
                ('/cmd_vel', '/Differential_drive_bot/cmd_vel'), 
            ]
        ),
        Node(
            package='my_planner',
            executable='tag_detector_node', 
            output='screen',
            name='tag_detector_node',
            remappings=[
                ('/tag_detections', '/apriltag_node/tag_detections') 
            ]
        )
    ])
