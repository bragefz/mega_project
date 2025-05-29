from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mega_project_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='example@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_node = mega_project_1.robot_control_node:main',
            'ur5e_control = mega_project_1.ur5e_control:main',
            'controller_manager = mega_project_1.movement_manager:main',
        ],
    },
)
