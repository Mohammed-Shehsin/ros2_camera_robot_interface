from setuptools import setup

package_name = 'camera_click_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Camera click teleop for TurtleBot',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable name      module path                        : function
            'click_teleop_node = camera_click_teleop.click_teleop_node:main',
        ],
    },
)
