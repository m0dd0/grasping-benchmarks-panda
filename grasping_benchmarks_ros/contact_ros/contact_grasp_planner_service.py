#!/usr/bin/env python3

from pathlib import Path
import logging
import importlib
import time
from typing import List
from uuid import uuid4

import yaml
import matplotlib as mpl
from PIL import Image

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
from contact_graspnet.utils.misc import setup_tensorflow
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

        setup_tensorflow()

        self._preprocessor = module_from_config(config["preprocessor"])
        self._postprocessor = module_from_config(config["postprocessor"])
        self._model = module_from_config(config["model"])
        self._cam2world_converter = module_from_config(config["cam2world_converter"])
        self._exporter = Exporter(debug_path)
        logging.info("Loaded modules from config")

    def _create_sample(self, req: GraspPlannerRequest) -> YCBSimulationDataSample:
        rgb = ros_numpy.numpify(req.color_image)
        depth = (ros_numpy.numpify(req.depth_image) / 1000).astype(np.float32)
        seg = ros_numpy.numpify(req.seg_image)
        camera_matrix = np.array(req.camera_info.K).reshape(3, 3)
        camera_trafo_h = ros_numpy.numpify(
            req.view_point.pose
        )  # 4x4 homogenous tranformation matrix

        sample = YCBSimulationDataSample(
            rgb=rgb,
            depth=depth,
            points=None,
            points_color=None,
            points_segmented=None,
            points_segmented_color=None,
            segmentation=seg,
            cam_intrinsics=camera_matrix,
            cam_pos=camera_trafo_h[:3, 3],
            cam_rot=camera_trafo_h[:3, :3],
            name=f"{time.strftime('%Y%m%d-%H%M%S')}__{uuid4()}",
        )

        return sample

    def _create_response(self, grasps: List[GraspWorld]) -> GraspPlannerResponse:
        response = GraspPlannerResponse()
        for g in grasps:
            grasp_msg = BenchmarkGrasp()

            pose = PoseStamped()
            # p.header.frame_id = self.ref_frame
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = g.position[0]
            pose.pose.position.y = g.position[1]
            pose.pose.position.z = g.position[2]

            quat = Rotation.from_matrix(g.orientation).as_quat()
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            grasp_msg.pose = pose

            grasp_msg.score.data = g.score
            grasp_msg.width.data = g.width

            response.grasp_candidates.append(grasp_msg)

        return response

    def _save_debug_information(
        self, grasps_cam, grasps_world, sample_rgb, full_pc, full_pc_colors, export_name
    ):
        export_data = {
            "grasps_cam": grasps_cam,
            "sample_rgb": Image.fromarray(sample_rgb),
            "full_pc": full_pc,
            "full_pc_colors": full_pc_colors,
            # "segmented_pc": segmented_pc,
            # "segmented_pc_colors": preprocessor.intermediate_results[
            #     "segmented_pc_colors"
            # ],
        }

        self._exporter(export_data, export_name)

    def srv_handler(self, req: GraspPlannerRequest) -> GraspPlannerResponse:
        logging.info("Received service call")

        sample = self._create_sample(req)
        logging.info(f"Processing datapoint: {sample}")

        full_pc, segmented_pc = self._preprocessor(sample)

        output = self._model(full_pc, segmented_pc)

        # the number of candidates to return is given in the service request
        # therefore we need to overwrrite the value in the postprocessor
        self._postprocessor.top_score_filter.top_k = req.n_of_candidates

        grasps_cam = self._postprocessor(output)
        logging.info(f"Found {len(grasps_cam)} grasps")

        grasps_world = [
            self._cam2world_converter(
                g_cam,
                sample.cam_pos,
                sample.cam_rot,
            )
            for g_cam in grasps_cam
        ]

        logging.info("saving debug information")
        self._save_debug_information(
            grasps_cam,
            grasps_world,
            sample.rgb,
            full_pc,
            self._preprocessor.intermediate_results["full_pc_colors"],
            sample.name,
        )

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
