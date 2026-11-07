from glob import glob

from setuptools import setup

setup(
    name='localizer',
    version='0.2.0',
    packages=['localizer', 'localizer.core'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/localizer']),
        ('share/localizer', ['package.xml']),
        ('share/localizer/launch', glob('launch/*.launch.py')),
        ('share/localizer/data', glob('data/*.bin')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ConnorG173',
    maintainer_email='connorg1@stanford.edu',
    description='Path follower: one ROS-free core, a simulator adapter and a real-drone adapter.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'real_follower = localizer.real:main',
            'sim_follower = localizer.sim:main',
        ],
    },
)
