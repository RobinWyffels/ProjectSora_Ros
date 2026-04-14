from setuptools import find_packages, setup

package_name = "sora_slam"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/sora_slam_toolbox.launch.py",
            "launch/sora_main_control.launch.py",
        ]),
        ("share/" + package_name + "/config", ["config/slam_toolbox_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="robin.wyffels@student.hogent.be",
    description="Sora mapping bringup (YDLidar + slam_toolbox).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "launch_control = sora_slam.launch_control:main",
        ],
    },
)