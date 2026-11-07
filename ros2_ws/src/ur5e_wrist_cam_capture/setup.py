from setuptools import find_packages, setup

package_name = "ur5e_wrist_cam_capture"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/poses.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sharath",
    maintainer_email="sharathnp1998@gmail.com",
    description="Capture wrist-camera images at fixed UR5e joint poses.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "capture_poses = ur5e_wrist_cam_capture.capture_poses:main",
            "project_check = ur5e_wrist_cam_capture.project_check:main",
        ],
    },
)