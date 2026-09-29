import os
from glob import glob
from setuptools import setup

package_name = "amr_bringup"


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
        data_files_for("config", "*.yaml"),
        data_files_for("urdf", "*.xacro"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Arun",
    maintainer_email="you@example.com",
    description="Real-robot bringup: L298N diff-drive motor node, sensors, EKF fusion (Pi)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_driver_node = amr_bringup.motor_driver_node:main",
            "mpu6050_node = amr_bringup.mpu6050_node:main",
        ],
    },
)
