from pathlib import Path
import setuptools


setuptools.setup(
    name="grasping_benchmarks",
    version="0.0.0",
    # author="elena rampone",
    # author_email="elena.rampone@iit.it",
    packages=setuptools.find_packages(),
    package_data={
        "grasping_benchmarks": list(Path(__file__).parent.glob("**/*.{yaml,xml,ini"))
    },
    python_requires=">=3",
    install_requires=["numpy", "nptyping"],  # requirements from the base classes
    extras_require={
        "dev": ["black", "pylint"],
        "grconvnet": [
            "pyyaml",
            "numpy",
            "ros_numpy @ git+https://github.com/m0dd0/ros_numpy@master",  # the public/original version is outdated and does not support new version of numpy
            "matplotlib",
            "scipy",
            "torch",
            "nptyping",
            "Pillow",
            "rospkg",
        ],
        "contact_graspnet": [],
    },
)
