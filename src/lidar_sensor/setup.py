from setuptools import find_packages, setup

package_name = 'lidar_sensor'

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
    maintainer='nablackfox',
    maintainer_email='nablackfox@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "straight_line = lidar_sensor.straight_line_detect:main",
            "hough_line = lidar_sensor.hough_line_move:main",
            "parallel_line_move = lidar_sensor.parallel_line_demo:main",
            "corridor_center = lidar_sensor.corridor_center:main",
            "median_filter = lidar_sensor.median_filter:main"
        ],
    },
)
