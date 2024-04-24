from typing import List, Dict

from grasping_benchmarks.base import BaseGraspPlanner, CameraData, Grasp6D

from se3dif.models.loader import load_model
from se3dif.samplers import Grasp_AnnealedLD
from se3dif.utils import to_numpy, to_torch
from se3dif.visualization import create_gripper_marker


class Se3DifGraspPlanner(BaseGraspPlanner):
    def __init__(
        self,
        cfg: Dict,
    ):
        super(Se3DifGraspPlanner, self).__init__(cfg)

        self._model = load_model(
            {
                "device": self.cfg["device"],
                "pretrained_model": self.cfg["pretrained_model"],
            }
        )
        self._generator = Grasp_AnnealedLD(
            self._model,
            batch=self.cfg["batch"],
            T=self.cfg["T"],
            T_fit=self.cfg["T_fit"],
            k_steps=self.cfg["k_steps"],
            device=self.cfg["device"],
        )

    def plan_grasps(
        self, camera_data: CameraData, n_candidates: int = 1
    ) -> List[Grasp6D]:
        """Computes the given number of grasp candidates from from the given
        camera data.

        Args:
            camera_data (CameraData): Contains the data to compute the grasp poses
            n_candidates (int, optional): The number of grasp candidates to compute. Defaults to 1.
        """
        self._camera_data = camera_data

        self._model.set_latent(
            to_torch(camera_data.pointcloud[None, ...], self.cfg["device"]),
            batch=self.cfg["batch"],
        )

        self._H_grasps = to_numpy(self._generator.sample())

        return self._H_grasps

    def visualize(self):
        """Plot the grasp poses"""
        raise NotImplementedError
        # scene = trimesh.Scene()
        # scene.add_geometry(self._mesh)

        # for H_grasp in self._H_grasps:
        #     scene.add_geometry(create_gripper_marker().apply_transform(H_grasp))

        # scene.show()
