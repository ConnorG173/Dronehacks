import os
from glob import glob

from setuptools import setup

package_name = "team_loc_matcher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DroneHacks drone 12 team",
    maintainer_email="jholli@stanford.edu",
    description="Wall-matching drift correction fixes for team_loc",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["wall_matcher = team_loc_matcher.wall_matcher_node:main"]},
)
