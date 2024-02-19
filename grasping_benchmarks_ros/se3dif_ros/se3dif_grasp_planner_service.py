#!/usr/bin/env python3

from pathlib import Path
import logging
import copy
import importlib

import yaml
import h5py
import trimesh
import scipy

import ros_numpy
from scipy.spatial.transform import Rotation
import numpy as np

import numpy as np
from se3dif.models.loader import load_model
from se3dif.samplers import ApproximatedGrasp_AnnealedLD, Grasp_AnnealedLD
from se3dif.utils import to_numpy, to_torch
from se3dif.visualization import create_gripper_marker

import rospy

from geometry_msgs.msg import PoseStamped
from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)
from grasping_benchmarks_ros.msg import BenchmarkGrasp


class Se3DifGraspPlannerService:
    def __init__(
        self,
        service_name: str,
        config_file: str,
        # debug_path: str,
    ):
        logging.info("Starting Se3DifGraspPlannerService")
        self._service = rospy.Service(service_name, GraspPlanner, self.srv_handler)

        with open(config_file, "r") as f:
            config = yaml.safe_load(f)
        self._config = config
        logging.info("Loaded config from %s", config_file)

        self._model = load_model(
            {
                "device": self._config["device"],
                "pretrained_model": self._config["model"],
            }
        )

    def _load_acronym_mesh(
        self, dataset_path: Path, object_class: str, grasp_uuid: str
    ) -> trimesh.Trimesh:
        assert object_class in [
            p.name for p in (dataset_path / "grasps").iterdir()
        ], f"Object class {object_class} not found in dataset path {dataset_path}/grasps"

        grasp_file_path = list(
            (dataset_path / "grasps" / object_class).glob(
                f"{object_class}_{grasp_uuid}*.h5"
            )
        )

        assert len(grasp_file_path) != 0, f"Grasp file not found: {grasp_file_path}"
        assert (
            len(grasp_file_path) < 2
        ), f"Multiple grasp files found: {grasp_file_path}"
        grasp_file_path = grasp_file_path[0]

        grasp_data = h5py.File(grasp_file_path, "r")
        mesh_scale = grasp_data["object"]["scale"][()]
        mesh_file_path = (
            dataset_path
            / "meshes"
            / grasp_data["object"]["file"][()].decode("utf-8")[len("meshes") + 1 :]
        )

        mesh = trimesh.load_mesh(mesh_file_path)
        if type(mesh) == trimesh.scene.scene.Scene:
            mesh = trimesh.util.concatenate(mesh.dump())
        mesh = mesh.apply_translation(-mesh.centroid)
        mesh = mesh.apply_scale(mesh_scale)

        return mesh

    def _get_pointcloud_for_inference(
        self,
        mesh,
        random_rotation: bool = False,
        scaling_factor: float = 8.0,
        n_points: int = 1000,
    ):
        mesh = copy.deepcopy(mesh)

        H_rot = np.eye(4)
        if random_rotation:
            H_rot[:3, :3] = scipy.spatial.transform.Rotation.random().as_matrix()
        mesh.apply_transform(H_rot)

        mesh.apply_scale(scaling_factor)

        pointcloud = mesh.sample(n_points)

        return pointcloud, mesh, H_rot

    def srv_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        logging.info("Received service call")

        pointcloud = req.cloud
        pointcloud = ros_numpy.numpify(pointcloud)

        mesh = self._load_acronym_mesh(Path("/home/data/"), "ScrewDriver", "28d")

        pointcloud, mesh_transformed, H_rot = self._get_pointcloud_for_inference(
            mesh, random_rotation=False, scaling_factor=8.0, n_points=1000
        )

        self._model.set_latent(
            to_torch(pointcloud[None, ...], self._config["device"]),
            batch=self._config["batch"],
        )
        generator = Grasp_AnnealedLD(
            self._model,
            batch=self._config["batch"],
            T=70,
            T_fit=50,
            k_steps=2,
            device=self._config["device"],
        )

        H_grasps = to_numpy(generator.sample())
        H_grasps_rescaled = H_grasps.copy()
        H_grasps_rescaled[:, :3, 3] /= 8.0

        response = GraspPlannerResponse()

        scene = trimesh.Scene()
        scene.add_geometry(mesh)

        for H_grasp in H_grasps_rescaled:
            scene.add_geometry(create_gripper_marker().apply_transform(H_grasp))

        scene.show()

        return response


if __name__ == "__main__":
    # the name we give here gets overwritten by the <node name=...> tag from the launch file
    rospy.init_node("se3dif_grasp_planner")

    # relaodinf the logging config is necessary due to ROS logging behavior: https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(level=logging.INFO)

    Se3DifGraspPlannerService(
        rospy.get_param("~grasp_planner_service_name"),
        Path(rospy.get_param("~config_file")),
        # Path(rospy.get_param("~debug_path")),
    )

    rospy.spin()
