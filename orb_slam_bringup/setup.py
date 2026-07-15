from setuptools import find_packages, setup

package_name = "orb_slam_bringup"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            [
                "config/tasterobot_bag.yaml",
                "config/read_only_bridge.yaml",
            ],
        ),
        (
            "share/" + package_name + "/launch",
            [
                "launch/bag_replay.launch.py",
                "launch/dashboard.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="duc",
    maintainer_email="duc@example.com",
    description=(
        "Bag replay, supplemental TF, TF audit, metrics, and launch "
        "for ORB-SLAM3 lidar mapping."
    ),
    license="GPL-3.0-or-later",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "odom_tf_adapter = orb_slam_bringup.odom_tf_adapter:main",
            "tf_audit = orb_slam_bringup.tf_audit:main",
            "metrics_recorder = orb_slam_bringup.metrics_recorder:main",
            "orb_slam_report_check = orb_slam_bringup.report:check_main",
            "orb_slam_compare_runs = orb_slam_bringup.report:compare_main",
        ],
    },
)
