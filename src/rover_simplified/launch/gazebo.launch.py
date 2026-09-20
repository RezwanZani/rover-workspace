import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Update package name to your new simplified package
    pkg_share = get_package_share_directory('rover_simplified')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')

    # 2. Generate the raw XML string command (Do not wrap in ParameterValue yet)
    robot_desc_command = Command(['xacro ', xacro_file])

    # 3. Robot State Publisher Node (Wrap with ParameterValue here)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_desc_command, value_type=str)}]
    )

    # 4. Include Gazebo Harmonic's custom world simulation
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'rover_world.sdf')
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 5. Spawn the robot into Gazebo (Pass the raw command directly)
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'rover_world', 
            '-topic', '/robot_description', 
            '-name', 'rover_robo', 
            '-z', '0.5'
        ],
        output='screen'
    )

    # 6. Bridge ROS 2 /cmd_vel and /gps/fix topics
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # For motor control (ROS 2 -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            
            # GPS Sensor (Gazebo -> ROS 2)
            '/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        gazebo_cmd,
        spawn_node,
        bridge_node
    ])