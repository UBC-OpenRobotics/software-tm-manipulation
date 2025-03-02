from setuptools import find_packages, setup

package_name = 'humble_gazebo_gym'

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
    maintainer='aexzhou',
    maintainer_email='alexzhou330@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'collector = data_collector.collector_script:main',
            # 'point_cloud = data_collector.collector_script:main',
        ],
    },
)
