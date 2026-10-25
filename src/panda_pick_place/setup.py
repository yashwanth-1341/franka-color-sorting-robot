from setuptools import setup

package_name = "panda_pick_place"

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
    description="Random scene generation, colour selection UI and obstacle-aware pick and place for the Panda",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pick_and_place = panda_pick_place.pick_and_place:main",
            "color_selector = panda_pick_place.color_selector:main",
        ],
    },
)
