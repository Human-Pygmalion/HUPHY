from glob import glob

from setuptools import find_packages, setup

package_name = "huphy_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament 가 패키지 목록을 만들 때 보는 빈 마커 파일임. 지우면 colcon 이
        # 이 패키지를 못 찾음.
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chanhue",
    maintainer_email="hwagchan@gmail.com",
    description="HUPHY 제어를 ROS 2 노드로 감쌈",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_node = huphy_ros.control_node:main",
            "policy_node = huphy_ros.policy_node:main",
            "teleop_node = huphy_ros.teleop_node:main",
            "gamepad_node = huphy_ros.gamepad_node:main",
        ],
    },
)
