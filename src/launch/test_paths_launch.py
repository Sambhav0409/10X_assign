import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Get Package Directories ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # --- Define File Paths ---
    # Correct path to the empty world file provided by Gazebo
    world_path = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')

    # Path to the TurtleBot3 URDF model
    urdf_path = os.path.join(pkg_turtlebot3_gazebo, 'urdf', 'turtlebot3_waffle_pi.urdf')

    # --- Create Launch Description ---
    return LaunchDescription([
        # 1. Launch Gazebo with the specified empty world
        # We use gazebo.launch.py and pass the .world file path to it.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # 2. Launch the Robot State Publisher
        # Publishes the robot's state (TF tree) from the URDF.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[urdf_path],
        ),

        # 3. Spawn the TurtleBot3 Model in Gazebo
        # Spawns the robot into the world from the /robot_description topic.
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'turtlebot3_waffle_pi',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.01'  # Spawn slightly above the ground
            ]
        ),

        # 4. Launch Your Custom Controller Node
        Node(
            package='turtlebot3_path_follow_cpp',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
        ),
    ])
