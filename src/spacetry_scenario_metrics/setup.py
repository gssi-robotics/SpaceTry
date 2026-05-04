from pathlib import Path
from setuptools import setup

package_name = "spacetry_scenario_metrics"

Path("resource").mkdir(exist_ok=True)
(Path("resource") / package_name).touch(exist_ok=True)

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SpaceTry",
    maintainer_email="ricardo.caldas@gssi.it",
    description="Shared event-centric metrics and report helpers for SpaceTry scenario drivers.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
