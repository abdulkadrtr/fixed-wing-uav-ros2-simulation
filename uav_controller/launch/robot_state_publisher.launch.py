from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os

curr_dir = os.path.dirname(__file__)
print(curr_dir)
parent_dir = os.path.dirname(curr_dir)
urdf_file = os.path.join(parent_dir, 'urdf', 'robot.urdf')
rviz2_config_file = os.path.join(parent_dir, 'config', 'config.rviz')
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', str(urdf_file)])}]
        ),
        Node(
            package='uav_controller',
            executable='uav_controller',
            name='uav_controller',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz2_config_file)]
        )
    ])