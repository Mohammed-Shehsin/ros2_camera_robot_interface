from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # === 1) TURTLEBOT3 IN GAZEBO ===
    tb3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(
                    get_package_share_directory('turtlebot3_gazebo')
                ) / 'launch' / 'turtlebot3_world.launch.py'
            )
        )
    )

    # === 2) LAPTOP CAMERA NODE (v4l2) ===
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
    )

    # === 3) CLICK TELEOP NODE ===
    teleop_node = Node(
        package='camera_click_teleop',
        executable='click_teleop_node',
        name='click_teleop',
        output='screen',
        parameters=[{
            'image_topic': '/image_raw',    # from v4l2_camera
            'cmd_vel_topic': '/cmd_vel',    # TurtleBot3
            'forward_speed': 0.15,
            'backward_speed': -0.15,
            'command_duration': 0.5,
        }],
    )

    return LaunchDescription([
        tb3_gazebo_launch,
        camera_node,
        teleop_node,
    ])
