from glob import glob

from setuptools import setup

package_name = "rebotarm_dance"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="monkeyking",
    maintainer_email="user@example.com",
    description="Task 2: dance performance for reBot Arm (offline-IK tracks, sim & real)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dance_node = rebotarm_dance.dance_node:main",
            "precompute_dance = rebotarm_dance.precompute_dance:main",
        ],
    },
)
