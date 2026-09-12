import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get the path to the package and URDF file
    pkg_share = get_package_share_directory('rover_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    # 2. Read the URDF file as a string
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. Define the joint_state_publisher_gui node for joint control
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 5. Define the RViz2 node for 3D visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 6. Return the LaunchDescription to execute all nodes
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])