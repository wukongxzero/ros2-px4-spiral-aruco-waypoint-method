from setuptools import setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/aruco.launch.py']),
        ('share/' + package_name, ['launch/spiral.launch.py']),
        ('share/' + package_name, ['launch/waypoint.launch.py'])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavan',
    maintainer_email='pavankushal15729@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spiral_node = drone_control.spiral_node:main",
            "aruco_node = drone_control.aruco_node:main",
            "waypoint_node = drone_controller.waypoint_node:main"
        ],
    },
)
