from setuptools import find_packages, setup

package_name = "my_xarm_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "rclpy",
        "moveit_py",
    ],
    zip_safe=True,
    maintainer="mugugu",
    maintainer_email="mugugujose@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xarm_motion_planner1 = my_xarm_control.xarm_motion_planner1:main",
        ]
    },
)
