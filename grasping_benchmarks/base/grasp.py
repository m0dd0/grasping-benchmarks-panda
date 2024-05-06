# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# This software may be modified and distributed under the terms of the
# LGPL-2.1+ license. See the accompanying LICENSE file for details.

from typing import Union, List, Tuple

import numpy as np

from grasping_benchmarks.base import transformations

ROS_AVAILABLE = True
try:
    from grasping_benchmarks_ros.msg import BenchmarkGrasp
    import rospy
    from geometry_msgs.msg import PoseStamped
except ImportError as e:
    ROS_AVAILABLE = False


def grasp_to_ros_msg(grasp) -> BenchmarkGrasp:
    grasp_msg = BenchmarkGrasp()

    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = grasp.position[0]
    p.pose.position.y = grasp.position[1]
    p.pose.position.z = grasp.position[2]
    p.pose.orientation.x = grasp.quaternion[0]
    p.pose.orientation.y = grasp.quaternion[1]
    p.pose.orientation.z = grasp.quaternion[2]
    p.pose.orientation.w = grasp.quaternion[3]
    grasp_msg.pose = p

    grasp_msg.score.data = grasp.score
    grasp_msg.width.data = grasp.width

    return grasp_msg


class Grasp6D(object):
    def __init__(
        self,
        position: np.ndarray = np.zeros(3),
        rotation: np.ndarray = np.eye(3),
        width: float = 0.0,
        score: float = 0.0,
        ref_frame: str = "camera",
    ):
        """Initializes a 6D cartesian grasp.

        Args:
            position (np.ndarray, optional): 3-entry position vector wrt camera frame. Defaults to np.zeros(3).
            rotation (np.ndarray, optional): x3 rotation matrix wrt camera frame. Defaults to np.eye(3).
            width (float, optional): Distance between the fingers in meters. Defaults to 0.0.
            score (float, optional): Prediction score of the grasp pose. Defaults to 0.0.
            ref_frame (str, optional): Frame of reference for camera that the grasp corresponds to. Defaults to "camera".
        """

        self._position = position
        self._rotation = rotation

        self._check_valid_position(self._position)
        self._check_valid_rotation(self._rotation)

        self.width = width
        self.ref_frame = ref_frame
        self.score = score

        # rotation expressed as quaternion
        self._quaternion = transformations.matrix_to_quaternion(self._rotation)

    @property
    def rotation(self):
        return self._rotation

    @rotation.setter
    def rotation(self, rotation: np.ndarray):
        if type(rotation) in (list, tuple):
            rotation = np.array(rotation).astype(np.float32)

        self._check_valid_rotation(rotation)
        self._rotation = rotation * 1.0  # FIXME ?

        self._quaternion = transformations.matrix_to_quaternion(rotation)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position: Union[np.ndarray, List, Tuple]):
        # Convert list to position array
        if type(position) in (list, tuple) and len(position) == 3:
            position = np.array([t for t in position]).astype(np.float32)

        self._check_valid_position(position)
        self._position = position.squeeze() * 1.0

    @property
    def quaternion(self):
        return self._quaternion

    @quaternion.setter
    def quaternion(self, quat: Union[np.ndarray, List, Tuple]):
        # Convert quaternions
        if len(quat) != 4 or np.abs(np.linalg.norm(quat) - 1.0) > 1e-3:
            raise ValueError("Invalid quaternion")

        self._quaternion = np.array([q for q in quat])
        rotation = transformations.quaternion_to_matrix(q)

        self._check_valid_rotation(rotation)
        self._rotation = rotation * 1.0

    def _check_valid_rotation(self, rotation: np.ndarray):
        """Checks that the given rotation matrix is valid and raises error if not.

        Args:
            rotation (np.ndarray): The candidate to check. (Also checks for the correct
                data type)
        """
        if not isinstance(rotation, np.ndarray) or not np.issubdtype(
            rotation.dtype, np.number
        ):
            raise ValueError("Rotation must be specified as numeric numpy array")

        if len(rotation.shape) != 2 or rotation.shape[0] != 3 or rotation.shape[1] != 3:
            raise ValueError("Rotation must be specified as a 3x3 ndarray")

        if np.abs(np.linalg.det(rotation) - 1.0) > 1e-3:
            raise ValueError("Illegal rotation. Must have determinant == 1.0")

    def _check_valid_position(self, position: np.ndarray):
        """Checks that the position vector is valid and raisses error if not.

        Args:
            position (np.ndarray): The candidate to check
        """
        if not isinstance(position, np.ndarray) or not np.issubdtype(
            position.dtype, np.number
        ):
            raise ValueError("Position must be specified as numeric numpy array")

        pos = position.squeeze()
        if len(pos.shape) != 1 or pos.shape[0] != 3:
            raise ValueError(
                "position must be specified as a 3-vector, 3x1 ndarray, or 1x3 ndarray"
            )

    def to_ros_message(self) -> "BenchmarkGrasp":
        if not ROS_AVAILABLE:
            raise ImportError("ROS is not available")

        grasp_msg = BenchmarkGrasp()

        p = PoseStamped()
        p.header.frame_id = self.ref_frame
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = self.position[0]
        p.pose.position.y = self.position[1]
        p.pose.position.z = self.position[2]
        p.pose.orientation.x = self.quaternion[0]
        p.pose.orientation.y = self.quaternion[1]
        p.pose.orientation.z = self.quaternion[2]
        p.pose.orientation.w = self.quaternion[3]
        grasp_msg.pose = p

        grasp_msg.score.data = self.score
        grasp_msg.width.data = self.width

        return grasp_msg
