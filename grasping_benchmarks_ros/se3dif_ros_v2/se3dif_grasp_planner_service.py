#!/usr/bin/env python3

import logging
from typing import List
import importlib
from pathlib import Path

import yaml
import rospy

from grasping_benchmarks.base import CameraData, BaseGraspPlanner
from grasping_benchmarks.base import Grasp6D
from grasping_benchmarks.base.ros_converters import (
    service_request_to_camera_data,
    grasp_data_to_service_response,
)
from grasping_benchmarks.se3dif.se3dif_grasp_planner import Se3DifGraspPlanner

from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)


class Se3DifGraspPlannerService(Se3DifGraspPlanner):
    def srv_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        logging.info("Received service call")

        camera_data: CameraData = service_request_to_camera_data(req)

        grasps: List[Grasp6D] = self.plan_grasps(camera_data)

        response = grasp_data_to_service_response(grasps)

        return response


if __name__ == "__main__":
    # the name we give here gets overwritten by the <node name=...> tag from the launch file
    rospy.init_node("grconvnet_grasp_planner")

    # relaodinf the logging config is necessary due to ROS logging behavior: https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(level=logging.INFO)

    config_file = rospy.get_param("~config_file")
    config = yaml.safe_load(config_file.read_text())
    logging.info("Using config file %s", config_file)

    service = Se3DifGraspPlannerService(config)
    rospy.Service(
        rospy.get_param("~grasp_planner_service_name"),
        GraspPlanner,
        service.srv_handler,
    )

    rospy.spin()
