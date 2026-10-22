from glob import glob
from setuptools import find_packages, setup

package_name = 'simple_planner'

setup(
    name=package_name,
    version='1.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/paths', glob('paths/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='Reference path planner and validator for F1TENTH AROLA.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_path_publisher = simple_planner.simple_path_node:main',
        ],
    },
)
