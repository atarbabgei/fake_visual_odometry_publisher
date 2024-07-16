from setuptools import find_packages, setup

package_name = 'fake_visual_odometry_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Atar Babgei',
    maintainer_email='atarbabgei@gmail.com',
    description='ROS2 package that publishes fake visual odometry data to the PX4 Autopilot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = fake_visual_odometry_publisher.publisher:main',
        ],
    },
)
