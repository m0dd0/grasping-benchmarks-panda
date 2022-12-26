# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

from typing import Dict, List
from abc import abstractmethod

import numpy as np

from grasping_benchmarks.base.grasp import Grasp6D


class CameraData:
    def __init__(
        self,
        rgb_image: np.ndarray = None,  # rgb_img
        depth_image: np.ndarray = None,  # depth_img
        pointcloud: np.ndarray = None,  # pc_image
        segmentation_image: np.ndarray = None,  # seg_image
        bounding_box=None, # bounding box around target object ?
        intrinsic_params=None,
        extrinsic_params=None,
    ):
        self.rgb_image = rgb_image
        self.depth_image = depth_image
        self.pointcloud = pointcloud
        self.segmentation_image = segmentation_image
        self.bounding_box = bounding_box
        self.intrinsic_params = intrinsic_params
        self.extrinsic_params = extrinsic_params

        if self.extrinsic_params is None:
            self.extrinsic_params = {
                "position": np.ndarray((3, 1), float), #empty array
                "rotation": np.eye(3), # identity matrix
            }


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
        self._grasp_poses = [] 
        # containes the best grasp candidate of the last plan_grasp call
        self._best_grasp = None
        # contains the camera data send to the last plan_grasp call 
        self._camera_data = CameraData()
        # self._grasp_offset = np.zeros(3)

    def reset(self):
        """Sets the GraspPlanner back to its inital state. Especially removes all
        graps poses.
        """
        self._grasp_poses = []
        self._best_grasp = None
        self._camera_data = CameraData()

    @abstractmethod
    def plan_grasp(self, camera_data: CameraData, n_candidates: int = 1):
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
    def grasp_poses(self):
        return self._grasp_poses

    # @grasp_poses.setter
    # def grasp_poses(self, grasp_poses: List[Grasp6D]):
    #     if not all([isinstance(p, Grasp6D) for p in grasp_poses]):
    #         raise ValueError(
    #             "Invalid grasp type. Must be `benchmark_grasping.grasp.Grasp6D`"
    #         )

    #     self._grasp_poses = grasp_poses

    @property
    def best_grasp(self):
        return self._best_grasp

    # @best_grasp.setter
    # def best_grasp(self, best_grasp: Grasp6D):
    #     if type(best_grasp) is not Grasp6D:
    #         raise ValueError(
    #             "Invalid grasp type. Must be `benchmark_grasping.grasp.Grasp6D`"
    #         )

    #     self._best_grasp = best_grasp

    @property
    def camera_data(self):
        return self._camera_data

    @property
    def cfg(self):
        return self._cfg