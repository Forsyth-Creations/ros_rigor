from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the namespace
    module_name = DeclareLaunchArgument(
        'module_name', default_value='swerve_a', description='Namespace for the swerve module'
    )

    # Declare the node
    swerve_module_node = Node(
        package='hermes_swerve_module',  # The package containing your swerve module
        executable='hermes_swerve_module',  # The executable to run
        name='swerve_module',  # Node name
        output='screen',  # Output to screen (useful for debugging)
        parameters=[{'module_name': LaunchConfiguration('module_name')}, {"use_sim_time" : True}],  # Pass the namespace as a parameter
    )

    # Launch description
    return LaunchDescription([
        module_name,
        swerve_module_node
    ])
