#!/usr/bin/env python3

from pathlib import Path
import logging
import importlib
from typing import List

import yaml
import trimesh

import numpy as np
from se3dif.visualization import create_gripper_marker

import rospy

from grasping_benchmarks.base import CameraData, service_request_to_camera_data
from grasping_benchmarks.base.grasp_data import Grasp6D, grasp_data_to_service_response
from grasping_benchmarks.se3dif import Se3DifGraspPlanner
from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)
from grasping_benchmarks_ros.msg import BenchmarkGrasp


class GraspPlannerService:
    def __init__(self, service_name: str, config_file: str):
        logging.info("Starting %s service", service_name)
        self._service = rospy.Service(service_name, GraspPlanner, self.srv_handler)

        with open(config_file, "r") as f:
            config = yaml.safe_load(f)
        self._config = config
        logging.info("Loaded config from %s", config_file)

        self._planner = Se3DifGraspPlanner(config)

    def srv_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        logging.info("Received service call")

        camera_data: CameraData = service_request_to_camera_data(req)

        grasps: List[Grasp6D] = self._planner.plan_grasps(camera_data)

        response = grasp_data_to_service_response(grasps)

        return response


if __name__ == "__main__":
    # the name we give here gets overwritten by the <node name=...> tag from the launch file
    rospy.init_node("se3dif_grasp_planner")

    # relaodinf the logging config is necessary due to ROS logging behavior: https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(level=logging.INFO)

    GraspPlannerService(
        rospy.get_param("~grasp_planner_service_name"),
        Path(rospy.get_param("~config_file")),
    )

    rospy.spin()
