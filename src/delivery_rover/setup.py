from setuptools import find_packages, setup
from glob import glob

package_name = 'delivery_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/map', glob('config/map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suraj',
    maintainer_email='imsuraj.sj@gmail.com',
    description='Autonomous delivery rover',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim = delivery_rover.sim_node:main',
            'vision = delivery_rover.vision_node:main',
            'mission = delivery_rover.mission_node:main',
            'make_map = delivery_rover.make_map:main',
        ],
    },
)
