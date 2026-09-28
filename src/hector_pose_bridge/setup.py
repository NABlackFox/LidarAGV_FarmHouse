from glob import glob
from setuptools import find_packages, setup

package_name = 'hector_pose_bridge'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='hpvutrananhminh',
    maintainer_email='minhvutran2003@gmail.com',

    description='Hector pose and synchronized scan bridge for SLAM Toolbox',
    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'pose_to_odom_tf = hector_pose_bridge.pose_to_odom_tf:main',
            'sync_scan_bridge = hector_pose_bridge.sync_scan_bridge:main',
        ],
    },
)
