from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_teleop'

setup(
    name=package_name,
    version='2.3.7',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='Darby Lim',
    author_email='thlim@robotis.com',
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Teleoperation nodes using keyboard or Xbox controller for TurtleBot3.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = turtlebot3_teleop.script.teleop_keyboard:main'
        ],
    },
)
