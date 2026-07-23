from glob import glob
from setuptools import find_packages, setup

package_name = "portfolio_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jeongeon Lee",
    maintainer_email="jeongeonlee92@gmail.com",
    description="Bringup and smoke-test utilities for an Isaac Sim ROS 2 portfolio.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "status_publisher = portfolio_bringup.status_publisher:main",
            "warehouse_dispatcher = portfolio_bringup.warehouse_dispatcher:main",
            "aero_drill_terminal = portfolio_bringup.aero_drill_terminal:main",
        ],
    },
)
