from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1) UR5 bringup (SIM OR REAL)
    # TODO: adjust this path to your course's UR5 launch file if needed.
    ur5_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(
                    get_package_share_directory('ur_bringup')
                ) / 'launch' / 'ur_control.launch.py'
            )
        )
    )

    # 2) Camera driver (same as before)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
    )

    # 3) UR5 click teleop node
    teleop_node = Node(
        package='camera_click_teleop',
        executable='ur5_click_teleop_node',
        name='ur5_click_teleop',
        output='screen',
        parameters=[{
            'image_topic': '/image_raw',  # from v4l2_camera
            'traj_topic': '/scaled_joint_trajectory_controller/joint_trajectory',
            'joint_name': 'shoulder_pan_joint',
            'step': 0.1,
            'time_to_reach': 1.0,
        }],
    )

    return LaunchDescription([
        ur5_launch,
        camera_node,
        teleop_node,
    ])
