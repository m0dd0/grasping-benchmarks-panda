#!/usr/bin/env python3

from pathlib import Path

import yaml

import rospy
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import ros_numpy

from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)
from grasping_benchmarks_ros.msg import BenchmarkGrasp

from grasping_benchmarks.base.base_grasp_planner import CameraData
from grasping_benchmarks.base.grasp import Grasp6D

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

        self.cv_bridge = CvBridge()

    def plan_grasp_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        camera_data = CameraData.from_grasp_planner_request(req)

        n_candidates = req.n_of_candidates if req.n_of_candidates else 1

        grasps = self.plan_grasp(
            camera_data,
            n_candidates=n_candidates,
        )

        grasps_ros = [self._6DGrasp_2_grasp_response(grasp) for grasp in grasps]

        response = GraspPlannerResponse()
        for g in grasps_ros:
            response.grasp_candidates.append(g)

        return response

    def _6DGrasp_2_grasp_response(self, grasp: Grasp6D):
        grasp_msg = BenchmarkGrasp()

        p = PoseStamped()
        p.header.frame_id = grasp.ref_frame
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = grasp.position[0]
        p.pose.position.y = grasp.position[1]
        p.pose.position.z = grasp.position[2]
        p.pose.orientation.x = grasp.quaternion[0]
        p.pose.orientation.y = grasp.quaternion[1]
        p.pose.orientation.z = grasp.quaternion[2]
        p.pose.orientation.w = grasp.quaternion[3]
        grasp_msg.pose = p

        grasp_msg.score.data = grasp.score
        grasp_msg.width.data = grasp.width

        return grasp_msg


if __name__ == "__main__":
    rospy.init_node("grconvnet_grasp_planner")

    # TODO make parameters from the config gile rosparameters

    GRConvNetGraspPlannerService.from_config_file(
        Path(rospy.get_param("~config_file")),
        rospy.get_param("~grasp_planner_service_name"),
        rospy.get_param("~grasp_planner_topic_name"),
    )

    rospy.spin()
