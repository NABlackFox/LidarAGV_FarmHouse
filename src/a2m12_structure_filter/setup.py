from setuptools import find_packages, setup

package_name = 'a2m12_structure_filter'

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
    maintainer_email='minhvutran2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
 'structure_filter_node = a2m12_structure_filter.structure_filter_node:main',
        ],
    },
)
