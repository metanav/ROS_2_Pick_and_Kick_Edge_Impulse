from setuptools import find_packages, setup
from glob import glob

package_name = 'ainex_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch' , ['launch/kinematics.launch.py']),
        ('share/' + package_name + '/config', ['config/init_pose.yaml']),
        ('share/' + package_name + '/config', ['config/servo_controller.yaml']),
        ('share/' + package_name + '/config', ['config/walking_params.yaml']),
        ('share/' + package_name + '/config/action_groups',  glob('config/action_groups/*.d6a')),
        ( 'lib/python3.8/site-packages/' + package_name, glob('ainex_kinematics/*.so')),
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
            'controller_node = ainex_kinematics.controller_node:main'
        ],
    },
)
