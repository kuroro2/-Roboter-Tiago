from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'coke_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'coke_detector = coke_detector.coke_detector:main',
            'arm_move = coke_detector.arm_move:main',
        ],
    },
)
