from glob import glob
import os
from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (os.path.join('share', package_name, 'path'), glob('path/*.csv')),
        (os.path.join('share', package_name), ['LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata, George Halim, Karim Shousha',
    maintainer_email='fam@awadlouis.com, georgehany064@gmail.com',
    description='Pure Pursuit controller for F1TENTH autonomous racing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main'
        ],
    },
)
