# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

from typing import Dict, List
from abc import abstractmethod
from dataclasses import dataclass

import numpy as np

from grasping_benchmarks.base.typing import NpArray

ROS_AVAILABLE = True
try:
    from grasping_benchmarks_ros.srv import GraspPlannerRequest
    import ros_numpy
except ImportError as e:
    ROS_AVAILABLE = False

from .grasp import Grasp6D


def service_request_to_camera_data(req: "GraspPlannerRequest"):
    rgb = ros_numpy.numpify(req.color_image)
    depth = ros_numpy.numpify(req.depth_image)
    seg = ros_numpy.numpify(req.seg_image)

    pc = ros_numpy.numpify(req.cloud)

    camera_matrix = np.array(req.camera_info.K).reshape(3, 3)

    # 4x4 homogenous tranformation matrix
    camera_trafo_h = ros_numpy.numpify(req.view_point.pose)

    camera_data = CameraData(
        rgb,
        depth,
        pc,  # TODO check format and convert if needed
        seg,
        camera_matrix,
        camera_trafo_h[:3, 3],
        camera_trafo_h[:3, :3],
    )


@dataclass
class CameraData:
    rgb_img: NpArray["H, W, 3", np.uint8] = None
    depth_img: NpArray["H, W", np.uint16] = None
    pointcloud: NpArray["N, 3", np.float32] = None
    seg_img: NpArray["H, W", np.uint8] = None
    cam_intrinsics: NpArray["3, 3", np.float32] = None
    cam_pos: NpArray["3", np.float32] = None
    cam_rot: NpArray["3, 3", np.float32] = None


class BaseGraspPlanner(object):
    """The base class for grasp planners."""

    def __init__(self, cfg: Dict):
        """Initializes the grasp planner.

        Args:
            cfg (Dict): Dictionary of configuration parameters. This allows to easily
                and unifrormly access all currently used configurations of the planner.
        """
        self._cfg = cfg
        # maintains a list of the lastly proposed grasp candidates
        self._grasp_poses: List[Grasp6D] = None
        # contains the camera data send to the last plan_grasp call
        self._camera_data = None

    def reset(self):
        """Sets the GraspPlanner back to its inital state. Especially removes all
        graps poses.
        """
        self._grasp_poses = None
        self._camera_data = None

    @abstractmethod
    def plan_grasp(
        self, camera_data: CameraData, n_candidates: int = 1
    ) -> List[Grasp6D]:
        """Computes the given number of grasp candidates from from the given
        camera data. When implementing this class it needs to set the self._best_grasp,
        self._camera_data and self._grasp_poses properties of its instance accordingly.

        Args:
            camera_data (CameraData): Contains the data to compute the grasp poses
            n_candidates (int, optional): The number of grasp candidates to compute. Defaults to 1.
        """
        raise NotImplementedError()

    @abstractmethod
    def visualize(self):
        """Plot the lastly computed grasp poses"""
        raise NotImplementedError()

    @property
    def grasp_poses(self) -> List[Grasp6D]:
        return self._grasp_poses

    @property
    def best_grasp(self):
        if self._grasp_poses is None:
            return None
        best_grasp = self._grasp_poses[0]
        for grasp in self._grasp_poses:
            if grasp.score > best_grasp.score:
                best_grasp = grasp
        return best_grasp

    @property
    def camera_data(self):
        return self._camera_data

    @property
    def cfg(self):
        return self._cfg
