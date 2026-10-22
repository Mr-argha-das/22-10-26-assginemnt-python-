from setuptools import setup

package_name = 'watchdog'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config',
            ['config/watchdog_params.yaml'],
        ),
        (
            'share/' + package_name + '/launch',
            [
                'launch/watchdog.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='ROS 2 watchdog node for F1TENTH system monitoring and safety',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'watchdog_node = watchdog.watchdog_node:main',
        ],
    },
)
