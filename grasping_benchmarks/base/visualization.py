from typing import List, Tuple

import numpy as np
import trimesh

from grasping_benchmarks.base import CameraData, Grasp
from grasping_benchmarks.base.transformations import (
    rotation_and_position_to_homogeneous_matrix,
)


def panda_gripper_trimesh(
    hand_height: float = 0.127,
    finger_height: float = 0.05,
    gripper_width: float = 0.105,
    segment_radius: float = 0.005,
    color: Tuple[int, int, int] = (0, 0, 0),
) -> trimesh.Trimesh:
    """Creates a simple trimesh representation of the Panda gripper.

    Args:
        hand_height (float, optional): The height of the hand. Defaults to 0.127.
        finger_height (float, optional): The height of the fingers. Defaults to 0.05.
        gripper_width (float, optional): The width of the gripper. Defaults to 0.105.
        segment_radius (float, optional): The radius of the segments. Defaults to 0.01.

    Returns:
        trimesh.Trimesh: The trimesh representation of the Panda gripper.
    """
    hand_segment = trimesh.creation.cylinder(
        radius=segment_radius,
        segment=(
            (0, 0, 0),
            (0, 0, hand_height - finger_height),
        ),
    )
    horizontal_segment = trimesh.creation.cylinder(
        radius=segment_radius,
        segment=(
            (0, -gripper_width / 2, hand_height - finger_height),
            (0, gripper_width / 2, hand_height - finger_height),
        ),
    )
    left_finger = trimesh.creation.cylinder(
        radius=segment_radius,
        segment=(
            (0, -gripper_width / 2, hand_height - finger_height),
            (0, -gripper_width / 2, hand_height),
        ),
    )
    right_finger = trimesh.creation.cylinder(
        radius=segment_radius,
        segment=(
            (0, gripper_width / 2, hand_height - finger_height),
            (0, gripper_width / 2, hand_height),
        ),
    )

    gripper = trimesh.util.concatenate(
        [hand_segment, horizontal_segment, left_finger, right_finger]
    )
    gripper.visual.face_colors = color

    return gripper


def visualize_grasp_pointcloud(
    camera_data: CameraData,
    grasps: List[Grasp],
    max_pointcloud_points: int = None,
    show_coordinate_system: bool = True,
) -> trimesh.Scene:
    """Visualizes the given grasp candidates in the given camera data.

    Args:
        camera_data (CameraData): Contains the data to visualize the grasp poses
        grasps (List[Grasp]): The grasp candidates to visualize
        max_pointcloud_points (int, optional): The maximum number of points to sample from
            the pointcloud. Defaults to all points.
        show_coordinate_system (bool, optional): Whether to show the coordinate system. Defaults to True.

    Returns:
        trimesh.Scene: The scene containing the camera data and the grasp candidates
    """

    scene = trimesh.Scene()

    # trimeshs pointloud functionality seems to be buggy: https://github.com/mikedh/trimesh/issues/946
    # scene.add_geometry(trimesh.points.PointCloud(camera_data.pointcloud))

    pointcloud = camera_data.pointcloud
    if max_pointcloud_points is not None:
        pointcloud = pointcloud.sample(max_pointcloud_points)

    for point in pointcloud:
        scene.add_geometry(trimesh.primitives.Sphere(radius=0.001, center=point))

    for grasp in grasps:
        gripper_trimesh = panda_gripper_trimesh()
        gripper_trimesh.apply_transform(
            rotation_and_position_to_homogeneous_matrix(grasp.rotation, grasp.position)
        )
        scene.add_geometry(gripper_trimesh)

    if show_coordinate_system:
        coordinate_system = trimesh.creation.axis(
            origin_size=0.04,
            axis_radius=0.01,
            axis_length=1,
        )
        scene.add_geometry(coordinate_system)

    return scene
