from setuptools import setup, find_packages

setup(
    name="fretish_agent",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "pyyaml>=6.0",
    ],
    entry_points={
        "console_scripts": [
            "fretish-agent=fretish_agent.cli:main",
        ],
    },
)
