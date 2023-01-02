"""_summary_
"""

from pathlib import Path
from typing import List

import yaml
import numpy as np
import torch
from PIL import Image

from grconvnet.utils.load_models import get_model_path
from grconvnet.inference import GenerativeResnet
from grconvnet.datatypes import Grasp as GrconvnetGrasp
from grconvnet.datatypes import CameraData as GrconvnetCameraData
from grconvnet.preprocessing import Preprocessor, VisualizationPreprocessor
from grconvnet.postprocessing import Postprocessor, Img2WorldConverter
from grconvnet import visualization

from grasping_benchmarks.base.base_grasp_planner import BaseGraspPlanner, CameraData
from grasping_benchmarks.base.grasp import Grasp6D


class GRConvNetGraspPlanner(BaseGraspPlanner):
    @classmethod
    def from_config_file(cls, config_file: Path) -> "GRConvNetGraspPlanner":
        """Creates a new instance of the GRConvNetGraspPlanner from a config file.

        Args:
            config_file (Path): Path to the yaml config file. The yaml file must
                contain the same parameters as the __init__ method.

        Returns:
            GRConvNetGraspPlanner: The new instance
        """
        with open(config_file, "r") as f:
            cfg = yaml.safe_load(f)

        return cls(**cfg)

    def __init__(
        self,
        model_name: str,
        version_name: str,
        device: str,
        segment_rgb: bool,
        preprocessing_resize: bool,
        postprocessing_n_candidates: int,
        postprocessing_width_scale: float,
        postprocessing_min_grasp_distance: int,
        postprocessing_quality_threshold: float,
        postprocessing_blur: bool,
        conversion_convert_angle: bool,
    ):
        """_summary_

        Args:
            model_name (str): _description_
            version_name (str): _description_
            device (str): _description_
            segment_rgb (bool): _description_
            preprocessing_resize (bool): _description_
            postprocessing_n_candidates (int): _description_
            postprocessing_width_scale (float): _description_
            postprocessing_min_grasp_distance (int): _description_
            postprocessing_quality_threshold (float): _description_
            postprocessing_blur (bool): _description_
            conversion_convert_angle (bool): _description_
        """
        cfg = {
            "model_name": model_name,
            "version_name": version_name,
            "device": device,
            "segment_rgb": segment_rgb,
            "preprocessing_resize": preprocessing_resize,
            "posprocessing_n_candidates": postprocessing_n_candidates,
            "postprocessing_width_scale": postprocessing_width_scale,
            "postprocessing_min_grasp_distance": postprocessing_min_grasp_distance,
            "postprocessing_quality_threshold": postprocessing_quality_threshold,
            "postprocessing_blur": postprocessing_blur,
            "conversion_convert_angle": conversion_convert_angle,
        }
        super(GRConvNetGraspPlanner, self).__init__(cfg)

        self.model_path = get_model_path(model_name, version_name)
        self.segment_rgb = segment_rgb
        self.device = device
        if device is None:
            self.device = (
                torch.device("cuda")
                if torch.cuda.is_available()
                else torch.device("cpu")
            )
        self.preprocess_resize = preprocessing_resize
        self.postprocess_n_candidates = postprocessing_n_candidates
        self.postprocess_width_scale = postprocessing_width_scale
        self.postprocess_min_grasp_distance = postprocessing_min_grasp_distance
        self.postprocess_quality_threshold = postprocessing_quality_threshold
        self.postprocess_blur = postprocessing_blur
        self.conversion_convert_angle = conversion_convert_angle

        # for visualization we keep this data
        self.quality_img = None
        self.angle_img = None
        self.width_img = None
        self.preprocessed_rgb = None
        self.image_grasps = None

    def _convert_grasp_from_6D(self, grasp: Grasp6D) -> GrconvnetGrasp:
        """Converts a Grasp6D grasp to a GRConvNet grasp

        Args:
            grasp (Grasp6D): The grasp to convert

        Returns:
            GrconvnetGrasp: The converted grasp
        """
        return GrconvnetGrasp(
            center=grasp.position,
            width=grasp.width,
            score=grasp.score,  # TODO find out scale of score and adjust if necessary
        )

    def _convert_grasp_to_6D(self, grasp: GrconvnetGrasp) -> Grasp6D:
        """Converts a GRConvNet grasp to a Grasp6D grasp

        Args:
            grasp (GrconvnetGrasp): The grasp to convert

        Returns:
            Grasp6D: The converted grasp
        """
        return Grasp6D(
            position=grasp.center,
            rotation=np.eye(
                3
            ),  # grasp is alway parallel to the z-axis, assumes that rotation is relative to unit z-axis # TODO: check this
            width=grasp.width,
            score=grasp.score,  # TODO find out scale of score and adjust if necessary
            ref_frame="world",
        )

    def plan_grasp(
        self, camera_data: CameraData, n_candidates: int = 1
    ) -> List[Grasp6D]:
        """Computes the given number of grasp candidates from from the given
        camera data.

        Args:
            camera_data (CameraData): Contains the data to compute the grasp poses
            n_candidates (int, optional): The number of grasp candidates to compute. Defaults to 1.
        """

        self._camera_data = camera_data

        rgb = camera_data.rgb_img
        depth = camera_data.depth_img
        cam_intrinsics = camera_data.intrinsic_params
        cam_pos = camera_data.extrinsic_params["position"]
        cam_rot = camera_data.extrinsic_params["rotation"]

        # set all pixels to white where the segmentation mask is 0
        mask = camera_data.seg_img
        rgb_seg = np.full(rgb.shape, 255)
        rgb_seg[mask] = rgb[mask]

        model = GenerativeResnet()
        model.load_state_dict(torch.jit.load(self.model_path).state_dict())
        model.to(self.device)

        sample = GrconvnetCameraData(
            Image.fromarray(np.uint8(rgb_seg if self.segment_rgb else rgb)),
            Image.fromarray(depth),
        )

        preprocessor = Preprocessor(resize=self.preprocess_resize)
        input_tensor = torch.unsqueeze(preprocessor(sample), 0).to(self.device)

        visualization_preprocessor = VisualizationPreprocessor(
            resize=self.preprocess_resize
        )
        preprocessed_rgb, _ = visualization_preprocessor(sample)

        with torch.no_grad():
            prediction = model(input_tensor)

        postprocessor = Postprocessor(
            blur=self.postprocess_blur,
            width_scale=self.postprocess_width_scale,
            min_distance_between_grasps=self.postprocess_min_grasp_distance,
            quality_threshold=self.postprocess_quality_threshold,
            n_grasps=self.postprocess_n_candidates,
        )
        grasps_img, quality_img, angle_img, width_img = postprocessor(prediction)

        img2world_converter = Img2WorldConverter(
            cam_intrinsics,
            cam_pos,
            cam_rot,
            tuple(np.asarray(rgb).shape[:2]),
            resized_in_preprocess=self.preprocess_resize,
        )
        grasps_world = [img2world_converter(g, np.asarray(depth)) for g in grasps_img]

        self._grasp_poses = [self._convert_grasp(grasp) for grasp in grasps_world]

        # safe some data for visualization
        self.quality_img = quality_img
        self.angle_img = angle_img
        self.width_img = width_img
        self.preprocessed_rgb = preprocessed_rgb
        self.image_grasps = grasps_img

        return self._grasp_poses

    def visualize(self):
        """Plot the grasp poses"""
        visualization.create_overview_fig(
            rgb_orig=self._camera_data.rgb_img,
            rgb_preprocessed=self.preprocessed_rgb,
            quality_img=self.quality_img,
            angle_img=self.angle_img,
            width_img=self.width_img,
            image_grasps=self.image_grasps,
            world_grasps=[self._convert_grasp_from_6D(g) for g in self._grasp_poses],
            camera_matrix=self._camera_data.intrinsic_params,
            camera_rotation=self._camera_data.extrinsic_params["rotation"],
            camera_position=self._camera_data.extrinsic_params["position"],
        )
