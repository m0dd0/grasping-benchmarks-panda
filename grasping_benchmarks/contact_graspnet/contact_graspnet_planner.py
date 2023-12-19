import sys
from pathlib import Path
import logging
from typing import List

import tensorflow.compat.v1 as tf
import yaml

# The contact graspnet repo is not packaged and uses relative imports, so we need to
# add it to the path manually.
CONTACT_GRASPNET_PATH = Path("/home/contact_graspnet")
sys.path.append(str(CONTACT_GRASPNET_PATH / "contact_graspnet"))

from contact_grasp_estimator import GraspEstimator
from data import load_available_input_data
import config_utils

# from visualization_utils import visualize_grasps, show_image


class ContactGraspnetPlanner:
    @classmethod
    def from_config_file(cls, config_file: Path = None) -> "ContactGraspnetPlanner":
        """Creates a new instance of the ContactGraspnetPlanner from a config file.

        Args:
            config_file (Path): Path to the yaml config file. The yaml file must
                contain the same parameters as the __init__ method. If None, the
                default config file is used wich is expected to be in a cfg folder
                next to this file and is called base_config.yaml.

        Returns:
            ContactGraspnetPlanner: The new instance
        """

        if config_file is None:
            config_file = Path(__file__).parent / "cfg" / "base_config.yaml"

        with open(config_file, "r") as f:
            cfg = yaml.safe_load(f)

        return cls(**cfg)

    def __init__(
        self,
        checkpoint_dir: str,
        forward_passes: int,
        local_regions: bool,
        filter_grasps: bool,
        skip_border_objects: bool,
        z_range: List[float],
    ):
        checkpoint_dir = CONTACT_GRASPNET_PATH / "checkpoints" / checkpoint_dir

        self._forward_passes = forward_passes
        self._local_regions = local_regions
        self._filter_grasps = filter_grasps
        self._skip_border_objects = skip_border_objects
        self._z_range = z_range

        logging.info("Build the model")
        global_config = config_utils.load_config(
            checkpoint_dir, batch_size=self._forward_passes, arg_configs=[]
        )
        self._grasp_estimator = GraspEstimator(global_config)
        self._grasp_estimator.build_network()

        logging.info("Create a session")
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        self.session = tf.Session(config=config)

        logging.info("Load weights")
        saver = tf.train.Saver(save_relative_paths=True)
        self._grasp_estimator.load_weights(
            self.session, saver, checkpoint_dir, mode="test"
        )

    # def plan_grasp(self, camera_data: CameraData, n_candidates: int = 10):
    def plan_grasp(self, path):
        logging.info("'Loading available input data'")

        pc_segments = {}
        segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(
            str(path), K=None
        )

        if segmap is None and (self._local_regions or self._filter_grasps):
            raise ValueError(
                "Need segmentation map to extract local regions or filter grasps"
            )

        if pc_full is None:
            print("Converting depth to point cloud(s)...")
            (
                pc_full,
                pc_segments,
                pc_colors,
            ) = self._grasp_estimator.extract_point_clouds(
                depth,
                cam_K,
                segmap=segmap,
                rgb=rgb,
                skip_border_objects=self._skip_border_objects,
                z_range=self._z_range,
            )

        print("Generating Grasps...")
        (
            pred_grasps_cam,
            scores,
            contact_pts,
            _,
        ) = self._grasp_estimator.predict_scene_grasps(
            self.session,
            pc_full,
            pc_segments=pc_segments,
            local_regions=self._local_regions,
            filter_grasps=self._filter_grasps,
            forward_passes=self._forward_passes,
        )

        print("pred_grasps_cam", pred_grasps_cam)
        print("scores", scores)
        print("contact_pts", contact_pts)

        # Visualize results
        # mpl.use("Qt5Agg")
        # show_image(rgb, segmap)
        # visualize_grasps(
        #     pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colors
        # )

        # if not glob.glob(input_paths):
        #     print("No files found: ", input_paths)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    planner = ContactGraspnetPlanner.from_config_file(
        Path(__file__).parent / "cfg" / "base_config.yaml"
    )
    planner.plan_grasp(CONTACT_GRASPNET_PATH / "test_data" / "7.npy")
