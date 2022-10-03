from setuptools import setup

from glob import glob  # change for being able to read a file from a standard location

package_name = 'gps_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch*.py')),   # added so that launch files will be copied globally
        ('share/' + package_name+ '/my_data', glob('my_data/pose_list.txt')),  # change for opening the data file
        ('share/' + package_name+ '/my_data/stockpile', glob('my_data/stockpile/*.txt')),
        ('share/' + package_name+ '/rviz', glob('rviz/*.rviz')),  # copy rviz config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carl',
    maintainer_email='carl.crane@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_simulator = gps_nav.vehicle_simulator:main',
            'route_pose_provider = gps_nav.route_pose_provider:main',
            'vehicle_controller = gps_nav.vehicle_controller:main',
            'goal_pose_creator = gps_nav.goal_pose_creator:main',
            'motion_spec_provider = gps_nav.motion_spec_provider:main',
        ],
    },
)
