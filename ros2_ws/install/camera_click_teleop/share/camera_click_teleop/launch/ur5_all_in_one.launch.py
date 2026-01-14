from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    image_topic = LaunchConfiguration('image_topic')
    trajectory_topic = LaunchConfiguration('trajectory_topic')
    delta_pan = LaunchConfiguration('delta_pan')
    delta_lift = LaunchConfiguration('delta_lift')
    time_to_reach = LaunchConfiguration('time_to_reach')

    # ---------- URSim (Docker) ----------
    start_ursim = ExecuteProcess(
        cmd=['ros2', 'run', 'ur_robot_driver', 'start_ursim.sh', '-m', 'ur5'],
        output='screen'
    )

    # ---------- UR Driver (included launch) ----------
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': launch_rviz
        }.items()
    )

    # ---------- Camera ----------
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
    )

    # ---------- Click Teleop ----------
    teleop_node = Node(
        package='camera_click_teleop',
        executable='ur5_click_teleop_node',
        name='ur5_click_teleop',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'trajectory_topic': trajectory_topic,
            'delta_pan': delta_pan,
            'delta_lift': delta_lift,
            'time_to_reach': time_to_reach,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.56.101'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),

        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument(
            'trajectory_topic',
            default_value='/scaled_joint_trajectory_controller/joint_trajectory'
        ),
        DeclareLaunchArgument('delta_pan', default_value='0.4'),
        DeclareLaunchArgument('delta_lift', default_value='0.25'),
        DeclareLaunchArgument('time_to_reach', default_value='1.0'),

        start_ursim,

        # URSim needs time to boot
        TimerAction(period=30.0, actions=[ur_driver_launch]),

        camera_node,
        teleop_node,
    ])
