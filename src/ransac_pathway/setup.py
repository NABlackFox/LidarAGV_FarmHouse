from setuptools import find_packages, setup

package_name = 'ransac_pathway'

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
    maintainer='hpvutrananhminh',
    maintainer_email='hpvutrananhminh@example.com',
    description='Live RANSAC local pathway detection from 2D LiDAR scan',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_ransac_path_node = ransac_pathway.local_ransac_path_node:main',
        ],
    },
)
