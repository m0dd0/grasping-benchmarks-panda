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
    install_requires=[  # requirements from the base classes
        "numpy",
        "ros_numpy @ git+https://github.com/m0dd0/ros_numpy@master",  # the original version is outdated and does not support new version of numpy
    ],
    extras_require={
        "dev": ["black", "pylint"],
        "grconvnet": [
            "pyyaml",
            "numpy",
            "matplotlib",
            "scipy",
            "torch",
            "nptyping",
            "Pillow",
            "rospkg",
        ],
        "contact_graspnet": [
            "tensorflow==2.11.0",
            "numpy",
            "opencv-python",
            "opencv-python-headless",
            "pyyaml==5.3.1",
            "Pillow",
            "trimesh",
            "scipy",
            "pyrender==0.1.43",
            "pyglet==1.5.9",
            "tqdm",
            "matplotlib",
            "nptyping",
            "appdirs",
            "tabulate",
            "scikit-image",
        ],
    },
)
