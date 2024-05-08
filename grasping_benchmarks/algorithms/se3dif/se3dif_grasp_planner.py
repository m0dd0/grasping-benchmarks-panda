from typing import List, Dict

import numpy as np

from grasping_benchmarks.base import BaseGraspPlanner, CameraData, Grasp

from se3dif.models.loader import load_model
from se3dif.samplers import Grasp_AnnealedLD
from se3dif.utils import to_numpy, to_torch


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
    ) -> List[Grasp]:
        """Computes the given number of grasp candidates from from the given
        camera data.

        Args:
            camera_data (CameraData): Contains the data to compute the grasp poses
            n_candidates (int, optional): The number of grasp candidates to compute. Defaults to 1.
        """
        # we dont want to modify the original camera data pointcloud, so we need to copy it
        pointcloud_inference = camera_data.pointcloud_segmented.copy()

        pointcloud_inference = pointcloud_inference[
            np.random.choice(
                len(pointcloud_inference), self.cfg["n_points"], replace=False
            )
        ]

        # center and scale pointcloud
        pointcloud_offset = np.mean(pointcloud_inference, axis=0)
        pointcloud_inference -= pointcloud_offset
        pointcloud_extents = np.max(pointcloud_inference, axis=0) - np.min(
            pointcloud_inference, axis=0
        )
        scale_factor = self.cfg["object_target_size"] / np.max(pointcloud_extents)
        pointcloud_inference *= scale_factor

        self._model.set_latent(
            to_torch(pointcloud_inference[None, ...], self.cfg["device"]),
            batch=self.cfg["batch"],
        )

        grasps_inference = [
            Grasp(
                position=H_grasp[:3, 3],
                rotation=H_grasp[:3, :3],
            )
            for H_grasp in to_numpy(self._generator.sample())
        ]

        # the rotation of the grasp is invariant to the pointcloud offset and scale
        # but the position is not. Therefore, we need to transform the position back
        # to the original pointcloud frame
        grasps_retransformed = [
            Grasp(
                position=grasp.position / scale_factor + pointcloud_offset,
                rotation=grasp.rotation,
            )
            for grasp in grasps_inference
        ]

        # in the se3

        return grasps_retransformed, grasps_inference, pointcloud_inference
