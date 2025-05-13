from setuptools import find_packages, setup

package_name = "mujoco_ros2_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="joshua",
    maintainer_email="2578269186@qq.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_ros2_py = mujoco_ros2_py.mujoco_ros2_py:main",
        ],
    },
)
