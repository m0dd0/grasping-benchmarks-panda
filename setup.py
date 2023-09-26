from pathlib import Path
import setuptools


# get all files in ./grasping_benchmarks which end with a config file ending
config_file_endings = [".xml", ".yaml", ".ini"]
config_files = filter(
    lambda f: f in config_file_endings,
    (Path(__file__).parent / "grasping_benchmarks").rglob("*"),
)
config_files = list(config_files)

setuptools.setup(
    name="grasping-benchmarks",
    version="0.0.0",
    # author="elena rampone",
    # author_email="elena.rampone@iit.it",
    packages=setuptools.find_packages(),
    package_data={"grasping_benchmarks": config_files},
    python_requires=">=3",
    install_requires=["numpy", "nptyping"],  # requirements from the base classes
    extras_require={
        "dev": ["black", "pylint"],
        "grconvnet": [
            "grconvnet @ git+https://github.com/m0dd0/robotic-grasping@develop#egg=grconvnet",
            "numpy<1.24",
            "ros_numpy",
            "nptyping",
            "pyyaml",
            "torch",
            "Pillow",
            "rospkg",
            "scipy",
        ],
    },
)
