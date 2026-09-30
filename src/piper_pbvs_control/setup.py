from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'piper_pbvs_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xie',
    maintainer_email='xie@todo.todo',
    description='Vision-guided MoveIt coarse positioning for Piper.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevator_sequence = '
            'piper_pbvs_control.elevator_sequence:main',
            'joint_zero_return = '
            'piper_pbvs_control.joint_zero_return:main',
            'pbvs_controller = '
            'piper_pbvs_control.pbvs_controller:main',
        ],
    },
)
