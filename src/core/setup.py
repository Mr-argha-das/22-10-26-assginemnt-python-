from setuptools import find_packages, setup

package_name = 'core'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/core.launch.py', 'launch/simulation.core.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='Core center of the simple AROLA example providing fundamental functionality',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_base_link_node = core.odom_to_base_link_node:main'
        ],
    },
)
