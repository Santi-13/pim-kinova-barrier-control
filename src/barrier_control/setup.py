from setuptools import find_packages, setup

package_name = 'barrier_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_control.py', 'launch/dual_gen3_control_launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanmaster',
    maintainer_email='s.penunuri@hotmail.com',
    description='TODO: Package description',
    license='Apache_2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_listener = barrier_control.pose_listener:main',
            'rigid_body_dynamics_controller = barrier_control.rigid_body_dynamics_controller:main',
            'state_listener = barrier_control.state_listener:main',
            'target_handler = barrier_control.target_handler:main',
        ],
    },
)
