import sys
from pathlib import Path
import os
import logging

import tensorflow.compat.v1 as tf
import yaml


# The contact graspnet repo completely messed up imports as they only import files within
# the same folder and do not use relative imports for them.
# This means that we have to add all the files in the contact_graspnet folder of the original
# repo to the path to be able to access the contents of the files directly
CONTACT_GRASPNET_PATH = Path("/home/contact_graspnet")
sys.path = [str(p) for p in CONTACT_GRASPNET_PATH.rglob("*")] + sys.path

from contact_grasp_estimator import GraspEstimator
from data import load_available_input_data
import config_utils


class ContactGraspnetPlanner:
    @classmethod
    def from_config_file(cls, config_file: Path) -> "ContactGraspnetPlanner":
        """Creates a new instance of the ContactGraspnetPlanner from a config file.

        Args:
            config_file (Path): Path to the yaml config file. The yaml file must
                contain the same parameters as the __init__ method.

        Returns:
            ContactGraspnetPlanner: The new instance
        """

        with open(config_file, "r") as f:
            cfg = yaml.safe_load(f)

        return cls(**cfg)

    def __init__(self, checkpoint_dir, forward_passes):
        logging.info("Loading 'global config'")
        global_config = config_utils.load_config(
            checkpoint_dir, batch_size=forward_passes, arg_configs=[]
        )

        logging.info("Build the model")
        grasp_estimator = GraspEstimator(global_config)
        grasp_estimator.build_network()

        logging.info("Add ops to save and restore all the variables.")
        saver = tf.train.Saver(save_relative_paths=True)

        logging.info("Create a session")
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        session = tf.Session(config=config)

        logging.info("Load weights")
        grasp_estimator.load_weights(session, saver, checkpoint_dir, mode="test")

        os.makedirs("results", exist_ok=True)

    # def plan_grasp(self, camera_data: CameraData, n_candidates: int = 10):
    def plan_grasp(self, path):
        pc_segments = {}
        segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(
            path, K=None
        )

        # if segmap is None and (local_regions or filter_grasps):
        #     raise ValueError('Need segmentation map to extract local regions or filter grasps')

        # if pc_full is None:
        #     print('Converting depth to point cloud(s)...')
        #     pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(depth, cam_K, segmap=segmap, rgb=rgb,
        #                                                                             skip_border_objects=skip_border_objects, z_range=z_range)

        # print('Generating Grasps...')
        # pred_grasps_cam, scores, contact_pts, _ = grasp_estimator.predict_scene_grasps(sess, pc_full, pc_segments=pc_segments,
        #                                                                                   local_regions=local_regions, filter_grasps=filter_grasps, forward_passes=forward_passes)

        # # Save results
        # np.savez('results/predictions_{}'.format(os.path.basename(p.replace('png','npz').replace('npy','npz'))),
        #           pred_grasps_cam=pred_grasps_cam, scores=scores, contact_pts=contact_pts)

        # # Visualize results
        # show_image(rgb, segmap)
        # visualize_grasps(pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colors)


if __name__ == "__main__":
    planner = ContactGraspnetPlanner.from_config_file(
        Path(__file__).parent / "cfg" / "base_config.yaml"
    )
    # planner.plan_grasp()
