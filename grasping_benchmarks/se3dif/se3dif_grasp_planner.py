"""_summary_
"""

from pathlib import Path
from typing import List
import copy

import yaml

from grasping_benchmarks.base import BaseGraspPlanner, CameraData, Grasp6D

from se3dif.models.loader import load_model


class Se3DifGraspPlanner(BaseGraspPlanner):
    @classmethod
    def from_config_file(cls, config_file: Path) -> "Se3DifGraspPlanner":
        """Creates a new instance of the GRConvNetGraspPlanner from a config file.

        Args:
            config_file (Path): Path to the yaml config file. The yaml file must
                contain the same parameters as the __init__ method.

        Returns:
            SE3DifGraspPlanner: The new instance
        """
        with open(config_file, "r") as f:
            cfg = yaml.safe_load(f)

        return cls(**cfg)

    def __init__(
        self,
        cfg: dict,
    ):
        super(Se3DifGraspPlanner, self).__init__(cfg)

    def plan_grasp(
        self, camera_data: CameraData, n_candidates: int = 1
    ) -> List[Grasp6D]:
        """Computes the given number of grasp candidates from from the given
        camera data.

        Args:
            camera_data (CameraData): Contains the data to compute the grasp poses
            n_candidates (int, optional): The number of grasp candidates to compute. Defaults to 1.
        """

        self._camera_data = camera_data

        pointcloud = copy.deepcopy(camera_data.pointcloud)
        pointcloud *= 8.0

        model = load_model({"device": DEVICE, "pretrained_model": MODEL})

        return self._grasp_poses

    def visualize(self):
        """Plot the grasp poses"""

        return fig
