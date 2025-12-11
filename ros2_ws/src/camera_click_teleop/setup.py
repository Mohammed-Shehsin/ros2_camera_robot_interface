from setuptools import setup

package_name = 'camera_click_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # needed so `ros2 pkg list` can see the package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # install package.xml
        ('share/' + package_name, ['package.xml']),
        # install launch files
        ('share/' + package_name + '/launch', [
            'launch/click_teleop_with_v4l2.launch.py',
            'launch/tb3_click_teleop.launch.py',
            'launch/ur5_click_teleop_with_v4l2.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Camera-based click teleoperation for TurtleBot and UR robots',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'click_teleop_node = camera_click_teleop.click_teleop_node:main',
            'ur5_click_teleop_node = camera_click_teleop.ur5_click_teleop_node:main',
        ],
    },
)
