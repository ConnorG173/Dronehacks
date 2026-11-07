from glob import glob

from setuptools import setup

PACKAGE = 'team_path_following'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=[PACKAGE],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stanford DroneHacks team',
    description='Velocity-setpoint path following for the skybeta flight interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mission_tracker = team_path_following.node:main',
    ]},
)
