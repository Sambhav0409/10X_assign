from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    tb3_pkg = get_package_share_directory("turtlebot3_gazebo")
    pkg_share = FindPackageShare("turtlebot3_path_follow_cpp")
    
    # Paths
    world_launch = os.path.join(tb3_pkg, "launch", "turtlebot3_world.launch.py")
    params_file = PathJoinSubstitution([pkg_share, "params", "control_params.yaml"])
    
    # Arguments
    path_type_arg = DeclareLaunchArgument(
        "path_type",
        default_value="scurve",
        description="Type of path: line, circle, scurve, turns",
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    return LaunchDescription([
        path_type_arg,
        use_sim_time_arg,
        
        # Launch Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),
        
        # Waypoint publisher
        Node(
            package="turtlebot3_path_follow_cpp",
            executable="waypoint_publisher",
            name="waypoint_publisher",
            output="screen",
            parameters=[
                params_file,
                {"path_type": LaunchConfiguration("path_type")},
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),
        
        # Path smoother
        Node(
            package="turtlebot3_path_follow_cpp",
            executable="path_smoother",
            name="path_smoother",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),
        
        # Trajectory generator
        Node(
            package="turtlebot3_path_follow_cpp",
            executable="trajectory_generator",
            name="trajectory_generator",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),
        
        # Pure pursuit controller
        Node(
            package="turtlebot3_path_follow_cpp",
            executable="pure_pursuit_controller",
            name="pure_pursuit_controller",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),
    ])
