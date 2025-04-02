from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription()
    
    depth_to_laser_node = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            remappings=[
                ('depth', '/depth_camera/image_raw'),
                ('depth_camera_info', '/depth_camera/camera_info'),
                ('scan', '/hermes/scan')
            ],
            parameters=[{
                'scan_height': 10,  # Adjust as needed
                'scan_time': .0003,  # Adjust to match camera frame rate
                'range_min': 1.0,
                'range_max': 10.0,
                'output_frame' : 'CameraVision',
                'use_sim_time': True
            }]
        )
    
    
    # ld.add_action(nav2_node)
    ld.add_action(depth_to_laser_node)
    
    return ld