from setuptools import setup
import os
from glob import glob

package_name = 'spot_operation_vision'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.csv')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Visão e detecção de objetos para operação do Spot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = spot_operation_vision.object_detector:main',
            'depth_processor = spot_operation_vision.depth_processor:main',
            'image_utils = spot_operation_vision.image_utils:main',
        ],
    },
) 