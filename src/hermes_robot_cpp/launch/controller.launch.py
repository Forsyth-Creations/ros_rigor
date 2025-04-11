from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    main_controller = Node(
            package='hermes_robot_cpp',
            executable='hermes_robot_cpp',
            name='hermes_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    
    swerve_module_a = Node(
            package='hermes_robot_cpp',
            executable='swerve_module',
            name='swerve_module_a',
            namespace ='swerve_a',
            output='screen',
            parameters=[{'use_sim_time': True}, {'module_name': 'swerve_a'}],
        )
    
    swerve_module_b = Node(
            package='hermes_robot_cpp',
            executable='swerve_module',
            name='swerve_module_b',
            namespace ='swerve_b',
            output='screen',
            parameters=[{'use_sim_time': True}, {'module_name': 'swerve_b'}],
        )
    
    swerve_module_c = Node(
            package='hermes_robot_cpp',
            executable='swerve_module',
            name='swerve_module_c',
            namespace ='swerve_c',
            output='screen',
            parameters=[{'use_sim_time': True}, {'module_name': 'swerve_c'}],
        )
    
    swerve_module_d = Node(
            package='hermes_robot_cpp',
            executable='swerve_module',
            name='swerve_module_d',
            namespace ='swerve_d',
            output='screen',
            parameters=[{'use_sim_time': True}, {'module_name': 'swerve_d'}, {'enable_logging': False}],
        )
    
    ld.add_action(main_controller)
    ld.add_action(swerve_module_a)
    ld.add_action(swerve_module_b)
    ld.add_action(swerve_module_c)
    ld.add_action(swerve_module_d)
    
    
    return ld