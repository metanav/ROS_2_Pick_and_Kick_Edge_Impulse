from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'find_and_kick_ball'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource',  glob(os.path.join('resource', '*.eim'))),
        ('share/' + package_name + '/launch',  ['launch/object_detection_node.launch.py', 'launch/find_and_kick_ball_node.launch.py']),
        ('share/' + package_name + '/config',  ['config/calib.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naveen',
    maintainer_email='naveen@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_node = find_and_kick_ball.object_detection_node:main',
            'image_bbox_node = find_and_kick_ball.image_bbox_node:main',
            'find_and_kick_ball_node = find_and_kick_ball.find_and_kick_ball_node:main'
        ],
    },
)
