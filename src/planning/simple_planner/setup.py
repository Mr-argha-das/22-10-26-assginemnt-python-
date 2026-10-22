from setuptools import find_packages, setup

package_name = 'simple_planner'

setup(
    name=package_name,
    version='1.2.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/simple_planner.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='Simple planner for autonomous navigation. It takes waypoints from a CSV file and publishes them as a ROS path message.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_path_publisher = simple_planner.simple_path_node:main',
        ],
    },
)
