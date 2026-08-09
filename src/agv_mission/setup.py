from setuptools import find_packages, setup
from glob import glob

package_name = 'agv_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name,['package.xml']),
        ('share/' + package_name + '/config',glob('config/*')),
        ('share/' + package_name + '/launch',glob('launch/*.py')),
        ('share/' + package_name + '/filters',glob('filters/*')),
        ('share/' + package_name + '/paths',glob('paths/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nablackfox',
    maintainer_email='thanhchem.pro@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_follow = agv_mission.waypoint_follow:main',
            'set_init_amcl_pose = agv_mission.set_init_amcl_pose:main',
            'path_follow = agv_mission.path_follow:main',
            'waypoint_saver = agv_mission.waypoint_saver:main',
            'route_follow = agv_mission.route_follow:main',
            'route_tester = agv_mission.route_tester:main'
        ],
    },
)
