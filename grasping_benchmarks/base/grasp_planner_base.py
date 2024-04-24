# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

from typing import Dict, List
from abc import abstractmethod

from dataclasses import dataclass

import numpy as np


@dataclass
class CameraData:
    rgb_img: np.ndarray = None  # NpArray["H, W, 3", np.uint8] = None
    depth_img: np.ndarray = None  # NpArray["H, W", np.uint16] = None
    pointcloud: np.ndarray = None  # NpArray["N, 3", np.float32] = None
    seg_img: np.ndarray = None  # NpArray["H, W", np.uint8] = None
    cam_intrinsics: np.ndarray = None  # NpArray["3, 3", np.float32] = None
    cam_pos: np.ndarray = None  # NpArray["3", np.float32] = None
    cam_rot: np.ndarray = None  # NpArray["3, 3", np.float32] = None


@dataclass
class Grasp:
    position: np.ndarray  # NpArray["3", np.float32] = np.zeros(3)
    rotation: np.ndarray  # NpArray["3, 3", np.float32] = np.eye(3)
    width: float = 0.0
    score: float = 0.0
    ref_frame: str = "camera"


class BaseGraspPlanner(object):
    """The base class for grasp planners."""

    def __init__(self, cfg: Dict):
        """Initializes the grasp planner.

        Args:
            cfg (Dict): Dictionary of configuration parameters. This allows to easily
                and unifrormly access all currently used configurations of the planner.
        """
        self._cfg = cfg

    @abstractmethod
    def plan_grasp(self, camera_data: CameraData, n_candidates: int = 1) -> List[Grasp]:
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
    def cfg(self):
        return self._cfg
