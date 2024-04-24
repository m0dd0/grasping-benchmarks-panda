#!/usr/bin/env python3

import logging
from typing import List
import importlib
from pathlib import Path

import yaml
import rospy

from grasping_benchmarks.base import CameraData
from grasping_benchmarks.base.ros_converters import (
    service_request_to_camera_data,
    grasp_data_to_service_response,
)
from grasping_benchmarks.base import Grasp

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

        self._planner = None  # TODO

    def srv_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        logging.info("Received service call")

        camera_data: CameraData = service_request_to_camera_data(req)

        grasps: List[Grasp] = self._planner.plan_grasps(camera_data)

        response = grasp_data_to_service_response(grasps)

        return response


if __name__ == "__main__":
    # the name we give here gets overwritten by the <node name=...> tag from the launch file
    rospy.init_node("unnamed_grasp_planner_node")

    # relaodinf the logging config is necessary due to ROS logging behavior: https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(level=logging.INFO)

    print(rospy.get_param("~algorithm"))
    # print(rospy.get_param("~config_file"))
    # GraspPlannerService(
    #    rospy.get_param("~grasp_planner_service_name"),
    #    Path(rospy.get_param("~config_file")),
    # )

    rospy.spin()
