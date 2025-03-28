from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the nodes for each swerve module
    swerve_module_a = Node(
        package="hermes_swerve_module",  # The package containing your swerve module
        executable="hermes_swerve_module",  # The executable to run
        name="swerve_module_a",  # Node name
        namespace="swerve_a",  # Namespace for the module
        output="screen",  # Output to screen (useful for debugging)
        parameters=[
            {"module_name": "swerve_a"},
            {"use_sim_time": True}  # Use simulation time if needed
        ],  # Pass the namespace as a parameter
    )

    swerve_module_b = Node(
        package="hermes_swerve_module",
        executable="hermes_swerve_module",
        name="swerve_module_b",
        namespace="swerve_b",
        output="screen",
        parameters=[
            {
                "module_name": "swerve_b",
            },
            {"use_sim_time": True}  # Use simulation time if needed
        ],  # Invert the drive motor
    )

    swerve_module_c = Node(
        package="hermes_swerve_module",
        executable="hermes_swerve_module",
        name="swerve_module_c",
        namespace="swerve_c",
        output="screen",
        parameters=[
            {
                "module_name": "swerve_c",
            },
            {"use_sim_time": True}  # Use simulation time if needed
        ],
    )

    swerve_module_d = Node(
        package="hermes_swerve_module",
        executable="hermes_swerve_module",
        name="swerve_module_d",
        namespace="swerve_d",
        output="screen",
        parameters=[
            {"module_name": "swerve_d"},
            {"use_sim_time": True}  # Use simulation time if needed
        ],
    )

    # Return the launch description
    return LaunchDescription(
        [
            swerve_module_a,
            swerve_module_b,
            swerve_module_c,
            swerve_module_d,
        ]
    )
