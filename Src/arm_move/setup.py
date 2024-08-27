from setuptools import find_packages, setup

package_name = 'arm_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayman',
    maintainer_email='ayman.mezghani@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'tiago_move_test = tiago_nodes.tiago_move_test:main',
             'ex_joint_goal = tiago_nodes.tiago_test:main',
             'ex_pose_goal = tiago_nodes.pose_goal:main',
             'ArmtoCan = tiago_nodes.sub_move:main',
             'move_arm = tiago_nodes.arm_move:main',
             'gripper = tiago_nodes.gripper:main',
             'move_arm_trashcan = tiago_nodes.arm_to_trash:main',
             'move_arm_down = tiago_nodes.arm_move_down:main',
        ],
    },
)
