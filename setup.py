from setuptools import find_packages, setup

package_name = "minimal_opcua_test"

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
        "python-snap7",
        "opcua",
    ],
    zip_safe=True,
    maintainer="mugugu",
    maintainer_email="mugugujose@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "minimal_opcua_node=minimal_opcua_test.minimal_opcua_node:main",
            "min_opcua_connect_only=minimal_opcua_test.min_opcua_connect_only:main",
        ],
    },
)
