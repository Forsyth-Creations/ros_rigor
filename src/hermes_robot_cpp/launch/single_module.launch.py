from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    
    swerve_module_a = Node(
            package='hermes_robot_cpp',
            executable='swerve_module',
            name='swerve_module_a',
            namespace ='swerve_a',
            output='screen',
            parameters=[{'use_sim_time': True}, {'module_name': 'swerve_a'}, {"enable_logging": True}],
        )
    
    ld.add_action(swerve_module_a)
    
    return ld