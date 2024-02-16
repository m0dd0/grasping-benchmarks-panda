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
    install_requires=[
        "numpy",
        "ros_numpy @ git+https://github.com/m0dd0/ros_numpy@master",  # the original version is outdated and does not support new version of numpy
        "nptyping",
    ],
    extras_require={
        "dev": ["black", "pylint"],
        "grconvnet": [
            "pyyaml",
            "numpy",
            "matplotlib",
            # "numpy<1.24",
            "rosnumpy",
            "scipy",
            "torch",
            "nptyping",
            "Pillow",
            "rospkg",
            # TODO check if https://github.com/m0dd0/GrConvNetBenchmark.git can be installed from here
        ],
        "contactgraspnet": [
            # TODO check if this can be successfully installed from the setup.py instead of the Dockerfile
            # "contact_graspnet @ git+https://github.com/m0dd0/ContactGraspnetBenchmark@develop",
        ],
        "se3dif": [
            # grasp_diffusion package gets installed in Dockerfile as we need to install dependencies of grasp_diffusion package manually to avoid conflicts
            # also it needs to be installed with the -e flag for some reason which is not possible to specify in the setup.py
            # "se3dif @ git+https://github.com/TheCamusean/grasp_diffusion@master",
        ],
    },
)
