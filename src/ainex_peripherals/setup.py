from setuptools import find_packages, setup

package_name = 'ainex_peripherals'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/head_camera.yaml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
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
            'rgb = ainex_peripherals.rgb:main'
        ],
    },
)
