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
        ('share/' + package_name + '/launch', [
            'launch/launch_control.py', 
            'launch/dual_gen3_control_launch.py', 
            'launch/view_robot_gazebo.launch.py'
            ]),
        ('share/' + package_name + '/worlds', ['worlds/my_world.sdf']),
        ('share/' + package_name + '/urdf', [
            'urdf/my_robot.urdf.xacro',
            'urdf/rrbot.xacro',
            ]),
        ('share/' + package_name + '/config', [
            'config/cart_controller_velocity.yaml',
            'config/rrbot_controllers.yaml',
            ]),
        ('share/' + package_name + '/scripts', [
            'scripts/fault_clearer_node.py'
            ]), 


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanmaster',
    maintainer_email='s.penunuri@hotmail.com',
    description='TODO: Package description',
    license='Apache_2.0',
    # tests_require=['pytest'],
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'rigid_body_dynamics_controller = barrier_control.rigid_body_dynamics_controller:main',
            'target_handler = barrier_control.target_handler:main',
            'kortex_dual_arm_node = barrier_control.kortex_dual_arm_node:main',
            'gripper_tcp_node = barrier_control.gripper_tcp_node:main',
            'fault_clearer = barrier_control.scripts.fault_clearer_node:main',
        ],
    },
)
