from typing import List
from pathlib import Path
import sys

from grasping_benchmarks.base import BaseGraspPlanner, CameraData, Grasp6D


class ContactGraspnetPlanner(BaseGraspPlanner):
    def __init__(self, contact_graspnet_repo_path: Path):
        sys.path.append(str(contact_graspnet_repo_path))
        # from contact_graspnet import ContactGraspnet

        # self.contact_graspnet = ContactGraspnet()

    def plan_grasp(
        self, camera_data: CameraData, n_candidates: int = 1
    ) -> List[Grasp6D]:
        pass
