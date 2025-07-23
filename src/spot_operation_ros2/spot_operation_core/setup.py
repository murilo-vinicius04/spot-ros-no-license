from setuptools import setup
import os
from glob import glob

package_name = 'spot_operation_core'

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
    description='Core da operação do Spot - gerenciamento de conexões, TF e estado',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_client_manager = spot_operation_core.robot_client_manager:main',
            'tf_manager = spot_operation_core.tf_manager:main',
            'operation_state_manager = spot_operation_core.operation_state_manager:main',
        ],
    },
) 