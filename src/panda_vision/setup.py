from setuptools import setup

package_name = "panda_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Overhead camera colour detection for the Panda pick-and-place demo",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "color_detector = panda_vision.color_detector:main",
        ],
    },
)
