#!/usr/bin/env python3

import logging
from typing import List
import importlib
from pathlib import Path

import yaml
import rospy

from grasping_benchmarks.base import CameraData, BaseGraspPlanner
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

        response = grasp_data_to_service_response(grasps)

        return response


if __name__ == "__main__":
    # the name we give here gets overwritten by the <node name=...> tag from the launch file
    rospy.init_node("unnamed_grasp_planner_node")

    # relaodinf the logging config is necessary due to ROS logging behavior: https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(level=logging.INFO)

    # get the planner class of the specified algorithm
    algorithm_name = rospy.get_param("~algorithm")
    if algorithm_name == "se3dif":
        from grasping_benchmarks.algorithms.se3dif import Se3DifGraspPlanner

        planner_class = Se3DifGraspPlanner
    elif algorithm_name == "grconvnet":
        raise NotImplemented
    # TODO ...
    else:
        raise ValueError(f"Unknown algorithm {algorithm_name}")

    # getting the config file
    config_file = rospy.get_param("~config_file")
    if not config_file:
        config_file = (
            Path(__file__).parent.parent.parent
            / "algorithms"
            / algorithm_name
            / "cfg"
            / "base_config.yaml"
        )
    config = yaml.safe_load(config_file.read_text())
    logging.info("Using config file %s", config_file)

    planner = planner_class(config)

    service = GraspPlannerService(planner, f"grasp_planner_service_{algorithm_name}")
