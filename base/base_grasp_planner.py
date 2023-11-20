from abc import ABC, abstractmethod
from typing import List

from .datatypes import CameraData, Grasp6D


class BaseGraspPlanner(ABC):
    @abstractmethod
    def plan_grasp(
        self, camera_data: CameraData, n_candidates: int = 1
    ) -> List[Grasp6D]:
        """Computes the given number of grasp candidates from from the given
        camera data.

        Args:
            camera_data (CameraData): Contains the data to compute the grasp poses
            n_candidates (int, optional): The number of grasp candidates to compute. Defaults to 1.

        Returns:
            List[Grasp6D]: The computed grasp candidates
        """
        raise NotImplementedError()
