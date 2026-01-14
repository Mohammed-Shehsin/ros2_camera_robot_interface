from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments (you can override from CLI)
    image_topic = LaunchConfiguration('image_topic')
    traj_topic = LaunchConfiguration('trajectory_topic')
    delta_pan = LaunchConfiguration('delta_pan')
    delta_lift = LaunchConfiguration('delta_lift')
    time_to_reach = LaunchConfiguration('time_to_reach')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('trajectory_topic', default_value='/scaled_joint_trajectory_controller/joint_trajectory'),
        DeclareLaunchArgument('delta_pan', default_value='0.4'),
        DeclareLaunchArgument('delta_lift', default_value='0.25'),
        DeclareLaunchArgument('time_to_reach', default_value='1.0'),

        # 1) Webcam driver
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
        ),

        # 2) Your UR5 click teleop node
        Node(
            package='camera_click_teleop',
            executable='ur5_click_teleop_node',
            name='ur5_click_teleop',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'trajectory_topic': traj_topic,
                'delta_pan': delta_pan,
                'delta_lift': delta_lift,
                'time_to_reach': time_to_reach,
            }],
        ),
    ])
