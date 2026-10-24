from glob import glob
from setuptools import find_packages, setup

setup(
    name="ur12e_slicer_bridge",
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ur12e_slicer_bridge"]),
        ("share/ur12e_slicer_bridge", ["package.xml"]),
        ("share/ur12e_slicer_bridge/launch", glob("launch/*.launch.py")),
        ("share/ur12e_slicer_bridge/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UR12e Slicer project",
    maintainer_email="maintainer@example.invalid",
    description="Atomic JSON bridge from Windows Slicer to ROS 2 UR12e mock hardware.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["file_bridge = ur12e_slicer_bridge.node:main"]},
)
