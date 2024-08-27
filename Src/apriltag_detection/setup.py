from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'apriltag_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
         (os.path.join('share', package_name, 'config'), glob('config/*.config.*')),
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
            'to_coke = apriltag_detection.to_coke:main',
            'to_trash_can = apriltag_detection.to_trash_can:main',
        ],
    },
)
