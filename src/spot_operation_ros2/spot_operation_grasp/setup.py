from setuptools import setup
import os
from glob import glob

package_name = 'spot_operation_grasp'

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
    description='Estratégias de grasp para o Spot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_manager = spot_operation_grasp.grasp_manager:main',
            'grasp_strategy_base = spot_operation_grasp.grasp_strategy_base:main',
            'yolo_grasp_strategy = spot_operation_grasp.yolo_grasp_strategy:main',
            'manipulation_client = spot_operation_grasp.manipulation_client:main',
        ],
    },
) 