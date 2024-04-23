# grasping-benchmarks-panda

## Algorithms
This repo contains wrappers for multiple grasp planning algorithms.
In the following a short description of the algorithms is given.

### Se3Diffusion
https://github.com/robotgradient/grasp_diffusion

#### 
TODO

## Docker Container
For most algorithms a Dockerfile has been created that can be used to build a docker container which runs the corresponding ROS-service.
The docker containers cotnain all the necessary dependecies for the algorithms to run and the alforithms themselves.
They also contain ROS and this repo.
All of them launch the corresponding ROS-service when the container is started.

### Dependesncies and setup.py file
Most of the times the external algorithms we benchmark are poorly packaged or use some exotic dependencies.
For this reason, it often fails to directly install them as a (extra) dependency of this repo by listing them in the `setup.py` file.
Therefore, we decided to include the installation of the dependencies in the Dockerfile of the corresponding algorithm and do not list them in the `setup.py` file.
So if you want to install the algorithms without using docker you need to follow the installation instructions of the respective algorithm.

### Additional Data
Some of the docker container require you to mount special data directories to the container beacause they can not be downloaded in an automated way from within the Dockerfile as they are not publically available or are too large.
Except from that, the docker containers are self-contained.
In general, the data directories can be placed anywhere on the host system.
However, the `docker-compose.yaml` file as well as the `devcontainer.json` files exspect the data directories to be placed in the `grasping-benchmarks-panda/docker/<name>/additional_data` folder.
The following data directories need to be mounted to the container:
- `se3dif`: TODO

### Running the Docker Containers
To make the ROS services available to the host system, the docker container exposes the containers need to be started with the `--network=host` flag.
Some containers also neeed to be granted acces to the graphics card of the host system by providing `--gpus all` to the `docker run` command.
More details on which options are recommended to successfully run the containers can be inferred from the `docker-compose.yaml` file.
This contains the recommended options for running the containers and also the recommended options for building the containers.
Use `docker-compose up -d <name>` to start the container with the name `<name>` in the background.
<!-- Note that you might need to change the source file path for mounting the additional data directories in the `docker-compose.yaml` file to the correct path on your system. -->
Note that when using the `docker-compose.yaml` file to run the containers you need to change the source file path for mounting the additional data directories in the `docker-compose.yaml` file to the correct path on your system or put the data directories in the correct path on your system.

### Building the Docker Containers
Most Dockerfiles in this repos can be configured with multiple build arguments to build the container with different options.
However, the used default values should be sufficient for most use cases.
The respective build arguments are also shown in the `docker-compose.yaml` file as well as in the `Makefile`.
Common build arguments are:
- `PARENT_IMAGE`: The base image to use for the container. Should not be changed.
- `IS_DEVCONTAINER_BUILD`: If set to `true` the container is built with the intention to be used as a devcontainer for VSCode.
- `BENCHMARK_REPO`: The url to the git repository that contains the desired version of this repo.
- `BENCHMARK_BRANCH`: The branch of the git repository that should be used.

### Devcontaierns
Some of the Dockerfiles in this repo can also be used as devcontainers for VSCode.
This is especially useful for developing the wrappers of the algorithms in this repo.
The devcontainer use the same Dockerfiles as the normal containers but instead of cloning this benchmark repo into the container they link the local copy of this repo into the container.
This behaviour is controlled by the `IS_DEVCONTAINER_BUILD` build argument. 
The run configuration for the devcontainer is stored in the `.devcontainer` folder in the root of this repo.
This includes the linking of the local copy of this repo into the container.
Note that you need to change the source filepath for mounting the additional data directories in the `.devcontainer.json` file to the correct path on your system or put the data in the correct expsected path on your system. (See above)
Also note, that when using the devcontainer the `CMD` of the Dockerfile is ignored.

## Architecture and Interfaces
The goal of this Repo is to provide a unified interface for multiple grasp planning algorithms.
For this reason, we define a common ROS message for the grasp requests and a common ROS service for the grasp planning algorithms.
Also python base classes are provided that should be used to implement the wrappers for the algorithms.
So if you want to add a new algorithm to this repo you should:
- first subclass `BaseGraspPlanner` and implement the abstract `plan_grasps` method and optionally the `__init__` method.
- write a ROS service which uses your subclass of `BaseGraspPlanner` to plan grasps.

### Configuration Files





## Essentials from the original README
Here the shortest version of instructions on how to use this modified version of the repo:

```bash
mkdir -p ~/catkin_ws/src

# Clone this Repo into the src folder
cd ~/catkin_ws/src
git clone git@github.com:nicolas-schreiber/grasping-benchmarks-panda.git

# Build the Repo (just creates the representations of the GraspRequest ROS Message and the GraspService)
# We prefer catkin build, but catkin_make should also work
catkin build
# or
# cd ~/catkin_ws
# catkin_make

# Build the docker containers
cd ~/catkin_ws/src/grasping-benchmarks-panda/docker
make USER_NAME=<your username here> dexnet

# Run the docker container
cd ..
bash run.sh nicolas_schreiber dexnet_container nicolas_schreiber/benchmark_dexnet

# ========= Further useful commands =============== 
# When running the docker container again you can simply call, this reuses the last docker container
bash run.sh nicolas_schreiber dexnet_container

# For any C++ or Python scripts to have access to the `BenchmarkGrasp.msg` the `GraspPlanner.srv` run following command:
source ~/catkin_ws/devel/setup.bash

# By calling this in a different terminal you can check what services are available:
rosservice list
```

So far, this repo includes support for:

| Algorithm | Documentation | Paper |
| --- | --- | --- |
**Dexnet** | [docs](https://berkeleyautomation.github.io/dex-net/)  | [paper](https://arxiv.org/pdf/1703.09312.pdf) |
**GPD** | [docs](https://github.com/atenpas/gpd) | [paper](https://arxiv.org/pdf/1706.09911.pdf) |
**Superquadrics-based grasp planner**  | [docs](https://github.com/robotology/superquadric-lib) | [paper](http://lornat75.github.io/papers/2017/vezzani-icra.pdf) |
**6DoF-GraspNet** | [docs](https://github.com/jsll/pytorch_6dof-graspnet) [[original repo]](https://github.com/NVlabs/6dof-graspnet) | [paper](https://arxiv.org/abs/1905.10520) |

Install the dependencies of the algorithm you want to benchmark. You can follow the instructions provided by the authors of each algorithm, or you can take a look at the Docker recipes and see how we did it ourselves. The currently supported algorithms are:
    - **Dexnet**:You need to install [gqcnn](https://berkeleyautomation.github.io/gqcnn/)
    - **GPD**: Follow [gpd](https://github.com/atenpas/gpd)
    - **Superquadrics-based grasp planner**: Follow [superquadric-lib](https://github.com/robotology/superquadric-lib). Note that you need to compile the python bindings.
    - **6DoF-GraspNet**: We used [this PyTorch implementation](https://github.com/jsll/pytorch_6dof-graspnet).