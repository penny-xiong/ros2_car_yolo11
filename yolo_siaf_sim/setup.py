from glob import glob
import os
from setuptools import find_packages, setup

package_name = "yolo_siaf_sim"


def model_data_files():
    """Preserve the Gazebo model directory tree in the installed package."""
    entries = []
    for root, _, files in os.walk("models"):
        if files:
            entries.append(
                ("share/" + package_name + "/" + root, [os.path.join(root, f) for f in files])
            )
    return entries

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ] + model_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="YOLO-SIAF maintainer",
    maintainer_email="maintainer@example.com",
    description="Nighttime RGB-IR pedestrian detection and braking simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "detector_node = yolo_siaf_sim.detector_node:main",
            "speed_controller = yolo_siaf_sim.speed_controller:main",
        ],
    },
)
