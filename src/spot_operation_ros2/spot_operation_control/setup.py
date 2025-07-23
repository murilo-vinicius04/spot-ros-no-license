from setuptools import setup
import os
from glob import glob

package_name = 'spot_operation_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Controle de movimento do Spot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_controller = spot_operation_control.spot_controller:main',
            'potential_field = spot_operation_control.potential_field:main',
            'moveit_manager = spot_operation_control.moveit_manager:main',
            'collision_detector = spot_operation_control.collision_detector:main',
        ],
    },
) 