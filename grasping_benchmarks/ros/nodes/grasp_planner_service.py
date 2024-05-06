#!/usr/bin/env python3

import logging
from typing import List
import importlib
from pathlib import Path

import yaml
import rospy
import numpy as np
import ros_numpy

from grasping_benchmarks.base import CameraData, BaseGraspPlanner, Grasp
from geometry_msgs.msg import PoseStamped
from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)
from grasping_benchmarks_ros.msg import BenchmarkGrasp


def grasp_to_ros_message(grasp) -> BenchmarkGrasp:
    grasp_msg = BenchmarkGrasp()

    pose = PoseStamped()
    pose.header.frame_id = grasp.ref_frame
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = grasp.position[0]
    pose.pose.position.y = grasp.position[1]
    pose.pose.position.z = grasp.position[2]
    pose.pose.orientation.x = grasp.quaternion[0]
    pose.pose.orientation.y = grasp.quaternion[1]
    pose.pose.orientation.z = grasp.quaternion[2]
    pose.pose.orientation.w = grasp.quaternion[3]
    grasp_msg.pose = pose

    grasp_msg.score.data = grasp.score
    grasp_msg.width.data = grasp.width

    return grasp_msg


def service_request_to_camera_data(req: GraspPlannerRequest):
    camera_pose = ros_numpy.numpify(req.view_point.pose)

    camera_data = CameraData(
        rgb_image=ros_numpy.numpify(req.color_image),
        depth_image=ros_numpy.numpify(req.depth_image),
        pointcloud=ros_numpy.numpify(req.cloud).view(np.float32).reshape(-1, 3).copy(),
        pointcloud_segmented=ros_numpy.numpify(req.pointcloud_segmented)
        .view(np.float32)
        .reshape(-1, 3)
        .copy(),
        segmentation_image=ros_numpy.numpify(req.seg_image),
        camera_intrinsics=np.array(req.camera_info.K).reshape(3, 3),
        camera_position=camera_pose[:3, 3],
        camera_rotation=camera_pose[:3, :3],
    )

    return camera_data


class GraspPlannerService:
    def __init__(self, planner_class: BaseGraspPlanner, service_name: str):
        """The GraspPlannerService is a thin wrapper around the rospy.Service class and the
        passed planner grasp in order to transform the interface of the planner class to a ROS service.

        Args:
            planner_class (BaseGraspPlanner): The grasp planner class that should be used to compute the grasps
            service_name (str): The name of the service
        """
        logging.info("Starting %s service", service_name)
        self._service = rospy.Service(service_name, GraspPlanner, self.srv_handler)

        self._planner = planner_class

    def srv_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        """The service handler that is called when a service request is received.
        It transforms the service request to the camera data format, calls the planner
        and transforms the results back to the service response format.

        Args:
            req (GraspPlannerRequest): The service request

        Returns:
            GraspPlannerResponse: The service response
        """
        logging.info("Received service call")

        camera_data: CameraData = service_request_to_camera_data(req)

        grasps: List[Grasp] = self._planner.plan_grasps(camera_data)

        response = GraspPlannerResponse()
        response.grasp_candidates = [grasp_to_ros_message(grasp) for grasp in grasps]

        return response


if __name__ == "__main__":
    # the name we give here gets overwritten by the <node name=...> tag from the launch file
    rospy.init_node("unnamed_grasp_planner_node")

    # relaodinf the logging config is necessary due to ROS logging behavior: https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(level=logging.INFO)

    # getting the planner class
    module_string = ".".join(rospy.get_param("planner_class").split(".")[:-1])
    class_string = rospy.get_param("planner_class").split(".")[-1]
    planner_class = getattr(importlib.import_module(module_string), class_string)

    algorithm_config = rospy.get_param("~")
    logging.info("Using algorithm config: %s", algorithm_config)

    planner = planner_class(algorithm_config)

    service = GraspPlannerService(planner, f"{class_string}_service")

    rospy.spin()
