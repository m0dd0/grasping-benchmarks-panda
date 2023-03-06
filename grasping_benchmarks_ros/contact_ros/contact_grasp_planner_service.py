#!/usr/bin/env python3

from pathlib import Path
import logging
import importlib
import time
from typing import List

import yaml
import matplotlib as mpl

import ros_numpy
from scipy.spatial.transform import Rotation
import numpy as np

import rospy

from geometry_msgs.msg import PoseStamped
from grasping_benchmarks_ros.srv import (
    GraspPlanner,
    GraspPlannerRequest,
    GraspPlannerResponse,
)
from grasping_benchmarks_ros.msg import BenchmarkGrasp

from contact_graspnet.datatypes import YCBSimulationDataSample, GraspWorld
from contact_graspnet.utils.config import module_from_config
from contact_graspnet.utils.export import Exporter

# from contact_graspnet.utils.visualization import mlab_pose_vis


mpl.use("Agg")


class ContactGraspNetPlannerService:
    def __init__(
        self,
        config_file: str,
        service_name: str,
        debug_path: str,
    ):
        logging.info("Starting ContactGraspNetGraspPlannerService")
        self._service = rospy.Service(service_name, GraspPlanner, self.srv_handler)

        with open(config_file, "r") as f:
            config = yaml.safe_load(f)
        logging.info("Loaded config from %s", config_file)

        self._preprocessor = module_from_config(config["preprocessor"])
        self._postprocessor = module_from_config(config["postprocessor"])
        self._model = module_from_config(config["model"])
        self._cam2world_converter = module_from_config(config["cam2world_converter"])
        self._exporter = Exporter(debug_path)
        logging.info("Loaded modules from config")

        self._device = config["model"]["device"]

    def _create_sample(self, req: GraspPlannerRequest) -> YCBSimulationDataSample:
        rgb = torch.from_numpy(ros_numpy.numpify(req.color_image)).permute(2, 0, 1)
        depth = torch.unsqueeze(
            torch.from_numpy(
                (ros_numpy.numpify(req.depth_image) / 1000).astype(np.float32)
            ),
            0,
        )
        seg = torch.unsqueeze(torch.from_numpy(ros_numpy.numpify(req.seg_image)), 0)
        # pc = torch.from_numpy(ros_numpy.numpify(req.cloud))
        pc = np.array([])
        camera_matrix = np.array(req.camera_info.K).reshape(3, 3)
        camera_trafo_h = ros_numpy.numpify(
            req.view_point.pose
        )  # 4x4 homogenous tranformation matrix

        sample = YCBSimulationDataSample(
            rgb=rgb,
            depth=depth,
            points=pc,
            segmentation=seg,
            cam_intrinsics=camera_matrix,
            cam_pos=camera_trafo_h[:3, 3],
            cam_rot=camera_trafo_h[:3, :3],
            name=time.strftime("%Y%m%d-%H%M%S"),
        )

        return sample

    def _create_response(self, grasps: List[GraspWorld]) -> GraspPlannerResponse:
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

    def _save_debug_information(self, sample, grasps_img, grasps_world):
        pass

    def srv_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        logging.info("Received service call")

        sample = self._create_sample(req)
        logging.info(f"Processing datapoint: {sample}")

        input_tensor = self._preprocessor(sample)
        input_tensor = input_tensor.to(self._device)
        input_tensor = input_tensor.unsqueeze(0)

        with torch.no_grad():
            output = self._model(input_tensor)

        # the number of candidates to return is given in the service request
        # therefore we need to overwrrite the value in the postprocessor
        self._postprocessor.grasp_localizer.grasps = req.n_of_candidates

        grasps_img = self._postprocessor(output)
        logging.info(f"Found {len(grasps_img)} grasps")

        grasps_world = [
            self._img2world_converter(
                g_img,
                sample.depth,
                sample.cam_intrinsics,
                sample.cam_rot,
                sample.cam_pos,
            )
            for g_img in grasps_img
        ]

        logging.info("saving debug information")
        self._save_debug_information(sample, grasps_img, grasps_world)

        response = self._create_response(grasps_world)
        logging.info("Created response")

        return response


if __name__ == "__main__":
    # the name we give here gets overwritten by the <node name=...> tag from the launch file
    rospy.init_node("contact_graspnet_grasp_planner")

    # relaodinf the logging config is necessary due to ROS logging behavior: https://github.com/ros/ros_comm/issues/1384
    importlib.reload(logging)
    logging.basicConfig(level=logging.INFO)

    ContactGraspNetPlannerService(
        Path(rospy.get_param("~config_file")),
        rospy.get_param("~grasp_planner_service_name"),
        Path(rospy.get_param("~debug_path")),
    )

    rospy.spin()
