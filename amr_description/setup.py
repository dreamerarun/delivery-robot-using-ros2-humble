import os
from glob import glob
from setuptools import setup

package_name = "amr_description"


def data_files_for(subdir, pattern="*"):
    return (
        os.path.join("share", package_name, subdir),
        glob(os.path.join(subdir, pattern))
    )


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        data_files_for("launch", "*.py"),
        data_files_for("urdf", "*.xacro"),
        data_files_for("rviz", "*.rviz"),
        data_files_for("worlds", "*.sdf"),
        data_files_for("config", "*.yaml"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arun",
    maintainer_email="you@example.com",
    description="Laptop-side RViz visualization + optional Gazebo sim of the AMR",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
