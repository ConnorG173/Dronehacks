from glob import glob

from setuptools import setup

setup(
    name='localizer',
    version='0.1.0',
    packages=['localizer'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/localizer']),
        ('share/localizer', ['package.xml']),
        ('share/localizer/launch', glob('launch/*.launch.py')),
        ('share/localizer/data', glob('data/*.npz') + glob('data/*.bin')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ConnorG173',
    maintainer_email='connorg1@stanford.edu',
    description='Course-frame localization and path streaming for skybeta.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'path_streamer = localizer.path_streamer:main',
            'course_matcher = localizer.course_matcher:main',
        ],
    },
)
