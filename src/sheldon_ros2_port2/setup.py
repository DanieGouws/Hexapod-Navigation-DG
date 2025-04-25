from setuptools import find_packages, setup

package_name = 'sheldon_ros2_port2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danie',
    maintainer_email='danie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "planner_v1 = sheldon_ros2_port2.PlannerV1:main",
            # "test_send_cmdvel = sheldon_ros2_port2.test_send_cmdvel:main",

            "planner = sheldon_ros2_port2.planner:main",
            "planner_v1 = sheldon_ros2_port2.PlannerV1:main",
            "test_planner = sheldon_ros2_port2.test_planner:main",
            "test_send_cmdvel = sheldon_ros2_port2.test_send_cmdvel:main",
            "ros2_serial_bridge = sheldon_ros2_port2.ros2_serial_bridge:main"
        ],
    },
)
