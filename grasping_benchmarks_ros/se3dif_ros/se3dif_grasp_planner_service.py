#!/usr/bin/env python3

from pathlib import Path
import logging
import importlib
import time
from typing import List
from uuid import uuid4

import yaml
import matplotlib as mpl

mpl.use("Agg")
from matplotlib import pyplot as plt
import ros_numpy
from scipy.spatial.transform import Rotation
import torch
import numpy as np

import rospy

from geometry_msgs.msg import PoseStamped
from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)
from grasping_benchmarks_ros.msg import BenchmarkGrasp


class Se3DifGraspPlannerService:
    def __init__(
        self,
        service_name: str,
        config_file: str,
        debug_path: str,
    ):
        logging.info("Starting Se3DifGraspPlannerService")
        self._service = rospy.Service(service_name, GraspPlanner, self.srv_handler)

        # with open(config_file, "r") as f:
        #     config = yaml.safe_load(f)
        # logging.info("Loaded config from %s", config_file)

    def _create_sample(self, req: GraspPlannerRequest):
        pass

    def _create_response(self, grasps) -> GraspPlannerResponse:
        response = GraspPlannerResponse()
        for g in grasps:
            grasp_msg = BenchmarkGrasp()

            pose = PoseStamped()
            # p.header.frame_id = self.ref_frame
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = g.center[0]
            pose.pose.position.y = g.center[1]
            pose.pose.position.z = g.center[2]

            # to let the robot gripper point to the flor we first need to rotate around the z-axis by 180°
            # the gripper axis is along the robot eef y axis but the grasp angle is given wrt. the x axis
            # therfore we need to add 90° to the grasp angle
            quat = Rotation.from_euler("xyz", [np.pi, 0, g.angle + np.pi / 2]).as_quat()
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            grasp_msg.pose = pose

            grasp_msg.score.data = g.quality
            grasp_msg.width.data = g.width

            response.grasp_candidates.append(grasp_msg)

        return response

    def _save_debug_information(self):
        pass

    def srv_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        logging.info("Received service call")

        # sample = self._create_sample(req)
        # logging.info(f"Processing datapoint: {sample}")

        # input_tensor = self._preprocessor(sample)
        # input_tensor = input_tensor.to(self._device)
        # input_tensor = input_tensor.unsqueeze(0)

        # with torch.no_grad():
        #     output = self._model(input_tensor)

        # # the number of candidates to return is given in the service request
        # # therefore we need to overwrrite the value in the postprocessor
        # self._postprocessor.grasp_localizer.grasps = req.n_of_candidates

        # grasps_img = self._postprocessor(output)
        # logging.info(f"Found {len(grasps_img)} grasps")

        # grasps_world = [
        #     self._img2world_converter(
        #         g_img,
        #         sample.depth,
        #         sample.cam_intrinsics,
        #         sample.cam_rot,
        #         sample.cam_pos,
        #     )
        #     for g_img in grasps_img
        # ]

        # logging.info("saving debug information")
        # self._save_debug_information(sample, grasps_img, grasps_world)

        # response = self._create_response(grasps_world)
        # logging.info("Created response")

        response = GraspPlannerResponse()

        return response


if __name__ == "__main__":
    # the name we give here gets overwritten by the <node name=...> tag from the launch file
    rospy.init_node("se3dif_grasp_planner")

    # relaodinf the logging config is necessary due to ROS logging behavior: https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(level=logging.INFO)

    Se3DifGraspPlannerService(
        rospy.get_param("~grasp_planner_service_name"),
        Path(rospy.get_param("~config_file")),
        Path(rospy.get_param("~debug_path")),
    )

    rospy.spin()
