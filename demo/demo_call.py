"""
"""

from typing import Tuple, Union
from pathlib import Path

import rospy
import numpy as np
import yaml
from nptyping import Float, NDArray, Shape

from ros_utils import create_grasp_planner_request
from vis_utils import visualize_grasp

# Importing Grasp Planner ROS Packages
from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)


def create_grasp_planner_request_from_demo_data(
    input_folder: Union[str, Path]
) -> Tuple[GraspPlannerRequest, np.array, np.array]:
    """Creates a GraspPlannerRequest from the data in the input_folder.
    Only one grasp candidate will be requested.
    The input folder must contain data like in the "example_data" folder.
    Also returns the sample point cloud data

    Args:
        input_folder (Union[str, Path]): The location of the folder from where the data for the
            request comes.

    Returns:
        Tuple[GraspPlannerRequest, np.array, np.array]: The ROS GraspPlannerRequest
            amd the pointcloud points and pointcloud colors.
    """
    # load sample image data
    seg_img = np.load(Path(input_folder) / "seg_img.npy", allow_pickle=True)
    rgb_img = np.load(Path(input_folder) / "rgb_img.npy", allow_pickle=True)
    depth_img = np.load(Path(input_folder) / "depth_img.npy", allow_pickle=True)

    # load sample point cloud data
    pc = np.load(Path(input_folder) / "pointcloud.npz", allow_pickle=True)
    pc_points = pc["pc_points"]
    pc_colors = pc["pc_colors"]

    # get sample camera parameters
    with open(Path(input_folder) / "cam_info.yaml", "r", encoding="utf-8") as stream:
        cam_info = yaml.safe_load(stream)
    cam_pos = np.array(cam_info["cam_pos"])
    cam_quat = np.array(cam_info["cam_quat"])
    cam_intrinsics = np.array(cam_info["cam_intrinsics"])
    cam_height = cam_info["cam_height"]
    cam_width = cam_info["cam_width"]

    return (
        create_grasp_planner_request(
            rgb_img,
            depth_img,
            seg_img,
            pc_points,
            pc_colors,
            cam_pos,
            cam_quat,
            cam_intrinsics,
            cam_height,
            cam_width,
            1,
        ),
        pc_points,
        pc_colors,
    )


def call_grasp_planner(
    input_folder: str, service_id: str
) -> Tuple[NDArray[Shape["3"], Float], NDArray[Shape["4"], Float]]:
    """_summary_

    Returns:
        _type_: _description_
    """

    # get the request from the sample data
    planner_req, pc_points, pc_colors = create_grasp_planner_request_from_demo_data(
        input_folder
    )

    rospy.wait_for_service(service_id, timeout=30.0)
    grasp_planner = rospy.ServiceProxy(service_id, GraspPlanner)

    try:
        # get the grasp plannet reply: this is a list of BenchMarkGrasp messages
        reply: GraspPlannerResponse = grasp_planner(planner_req)
        # print(f"Service {grasp_planner.resolved_name} reply is: \n{reply}")
        grasp = reply.grasp_candidates[0]

        pose = grasp.pose.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat = [
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]
        width = grasp.width.data

        return position, quat, width, pc_points, pc_colors

    except rospy.ServiceException as e:
        print(f"Service {grasp_planner.resolved_name} call failed: {e}")


def main():
    # ROS Init
    rospy.init_node("test")

    graspnet_service = "/graspnet_bench/graspnet_grasp_planner_service"
    superquadrics_service = "/superquadric_bench/superq_grasp_planner_service"
    gpd_service = "/gpd_bench/gpd_grasp_planner/gpd_grasp_planner_service"
    dexnet_service = "/dexnet_bench/dexnet_grasp_planner_service"
    
    # Calling the grasp planner
    pos, quat = call_grasp_planner(
        Path(__file__).parent / "example_data", service_id=superquadrics_service
    )

    # visualization seems to be broken
    # visualize_grasp(pc_points, pc_colors, position, quat, width)

if __name__ == "__main__":
    main()
