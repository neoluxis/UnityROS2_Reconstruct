from setuptools import find_packages, setup
from glob import glob
import os

package_name = "image_display"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test", "launch"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="neolux",
    maintainer_email="neolux_lee@outlook.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_display = image_display.image_display:main",
            "image_publisher = image_display.image_publisher:main",
            "gui = image_display.gui:main",
        ],
    },
)
