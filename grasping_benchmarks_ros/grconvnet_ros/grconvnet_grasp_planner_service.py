#!/usr/bin/env python3

from pathlib import Path

import numpy as np

from alr_sim.utils.geometric_transformation import (
    quat2mat,
)  # TODO use transformation from this package

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
    def __init__(
        self, config_file: Path, grasp_service_name: str, grasp_planner_topic_name: str
    ):
        GRConvNetGraspPlanner.from_config_file(config_file)
        # super().__init__() # is called by GRConvNetGraspPlanner.from_config_file(config_file)

        # Initialize the ROS service
        self._grasp_planning_service = rospy.Service(
            grasp_service_name, GraspPlanner, self.plan_grasp_handler
        )

        # TODO: implement publisher

        self.cv_bridge = CvBridge()

    def req_to_cam_data(self, req: GraspPlannerRequest) -> CameraData:
        rgb = ros_numpy.numpify(req.color_image)
        depth = ros_numpy.numpify(req.depth_image)
        seg = ros_numpy.numpify(req.segmentation_image)

        camera_matrix = req.camera_info.K

        camera_pos = np.asarray(req.view_point.position)
        camera_quat = np.asarray(req.view_point.orientation)

        camera_data = CameraData(
            rgb, depth, seg, camera_matrix, camera_pos, quat2mat(camera_quat)
        )

        return camera_data

    def plan_grasp_handler(self, req: GraspPlannerRequest):
        camera_data = self.req_to_cam_data(req)

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

    GRConvNetGraspPlannerService(
        Path(rospy.get_param("~config_file")),
        rospy.get_param("~grasp_planner_service_name"),
        rospy.get_param("~grasp_planner_topic_name"),
    )

    rospy.spin()
