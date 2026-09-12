import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get package path and URDF file
    pkg_share = get_package_share_directory('rover_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. Robot State Publisher Node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Include Gazebo Harmonic's empty world simulation
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 4. Spawn the robot into Gazebo
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc, '-name', 'rover', '-z', '0.5'],
        output='screen'
    )

    # 5. Bridge ROS 2 /cmd_vel topic to Gazebo
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        gazebo_cmd,
        spawn_node,
        bridge_node
    ])