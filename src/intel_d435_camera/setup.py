from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'intel_d435_camera'

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
    maintainer='mark',
    maintainer_email='mark0975272753@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'camera_pub = {package_name}.camera_pub:main',
            f'camera_sub = {package_name}.camera_sub:main',
        ],
    },
)
