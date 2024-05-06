from typing import List

import trimesh

from grasping_benchmarks.base import CameraData, Grasp
from grasping_benchmarks.base.transformations import (
    rotation_and_position_to_homogeneous_matrix,
)


def panda_gripper_trimesh(
    hand_height=0.127, finger_height=0.05, gripper_width=0.105, segment_radius=0.01
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
        raius=segment_radius,
        segment=(
            (0, 0, 0),
            (0, 0, hand_height - finger_height),
        ),
    )
    horizontal_segment = trimesh.creation.cyliner(
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

    return trimesh.util.concatenate(
        [hand_segment, horizontal_segment, left_finger, right_finger]
    )


def visualize_grasp_pointcloud(
    camera_data: CameraData, grasps: List[Grasp]
) -> trimesh.Scene:
    """Visualizes the given grasp candidates in the given camera data.

    Args:
        camera_data (CameraData): Contains the data to visualize the grasp poses
        grasps (List[Grasp]): The grasp candidates to visualize

    Returns:
        trimesh.Scene: The scene containing the camera data and the grasp candidates
    """

    scene = trimesh.Scene()

    for point in camera_data.pointcloud:
        scene.add_geometry(trimesh.primitives.Sphere(radius=0.02, center=point))

    for grasp in grasps:
        gripper_trimesh = panda_gripper_trimesh()
        gripper_trimesh.apply_transform(
            rotation_and_position_to_homogeneous_matrix(grasp.rotation, grasp.position)
        )
        scene.add_geometry(gripper_trimesh)

    return scene
