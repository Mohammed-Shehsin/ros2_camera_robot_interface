from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        # publishes sensor_msgs/Image on /image_raw
    )

    ur5_teleop_node = Node(
        package='camera_click_teleop',
        executable='ur5_click_teleop_node',
        name='ur5_click_teleop',
        output='screen',
        parameters=[{
            'image_topic': '/image_raw',  # from v4l2_camera
            # UR5 controller topic:
            'trajectory_topic': '/scaled_joint_trajectory_controller/joint_trajectory',
            'base_joint_name': 'shoulder_pan_joint',
            'delta_angle': 0.1,
            'time_to_reach': 1.0,
        }],
    )

    return LaunchDescription([
        camera_node,
        ur5_teleop_node,
    ])
