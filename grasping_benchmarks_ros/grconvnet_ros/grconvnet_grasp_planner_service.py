#!/usr/bin/env python3

from pathlib import Path

import yaml

import rospy

from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)

from grasping_benchmarks.base import CameraData

from grasping_benchmarks.grconvnet.grconvnet_grasp_planner import GRConvNetGraspPlanner


class GRConvNetGraspPlannerService(GRConvNetGraspPlanner):
    @classmethod
    def from_config_file(
        cls, config_file: Path, grasp_service_name: str, grasp_planner_topic_name: str
    ) -> "GRConvNetGraspPlanner":
        """Creates a new instance of the GRConvNetGraspPlanner from a config file.

        Args:
            config_file (Path): Path to the yaml config file. The yaml file must
                contain the same parameters as the __init__ method.

        Returns:
            GRConvNetGraspPlanner: The new instance
        """
        with open(config_file, "r") as f:
            cfg = yaml.safe_load(f)

        return cls(grasp_service_name, grasp_planner_topic_name, **cfg)

    def __init__(
        self, grasp_service_name: str, grasp_planner_topic_name: str, *args, **kwargs
    ):
        super().__init__(*args, **kwargs)

        # Initialize the ROS service
        self._grasp_planning_service = rospy.Service(
            grasp_service_name, GraspPlanner, self.plan_grasp_handler
        )

        # TODO: implement publisher

    def plan_grasp_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        camera_data = CameraData.from_grasp_planner_request(req)

        n_candidates = req.n_of_candidates if req.n_of_candidates else 1

        grasps = self.plan_grasp(
            camera_data,
            n_candidates=n_candidates,
        )

        response = GraspPlannerResponse()
        for g in grasps:
            response.grasp_candidates.append(g.to_ros_message())

        return response


if __name__ == "__main__":
    rospy.init_node("grconvnet_grasp_planner")

    # TODO make parameters from the config gile rosparameters

    GRConvNetGraspPlannerService.from_config_file(
        Path(rospy.get_param("~config_file")),
        rospy.get_param("~grasp_planner_service_name"),
        rospy.get_param("~grasp_planner_topic_name"),
    )

    rospy.spin()
