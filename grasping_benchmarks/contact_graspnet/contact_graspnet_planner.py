from pathlib import Path

class ContactGraspnetModel:
    def __init__(self, config_path: Path, checkpoint_dir: Path, batch_size=1):
        super().__init__()

        config_path = Path(config_path).expanduser()
        checkpoint_dir = Path(checkpoint_dir).expanduser()

        checkpoint_dir = exists_in_subfolder(
            checkpoint_dir, get_root_dir() / "checkpoints"
        )
        config_path = exists_in_subfolder(config_path, checkpoint_dir)

        # TODO recator grasp estimator

        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        config["OPTIMIZER"]["batch_size"] = batch_size

        self._grasp_estimator = GraspEstimator(config)
        self._grasp_estimator.build_network()

        # Add ops to save and restore all the variables.
        saver = tf.train.Saver(save_relative_paths=True)

        # Create a session
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        self._sess = tf.Session(config=config)

        self._grasp_estimator.load_weights(
            self._sess, saver, checkpoint_dir, mode="test"
        )

    def __call__(
        self,
        pc_full: NDArray[Shape["N, 3"], Float],
        pc_segment: NDArray[Shape["N, 3"], Float] = None,
    ) -> Tuple[
        NDArray[Shape["N,4,4"], Float],
        NDArray[Shape["N"], Float],
        NDArray[Shape["N, 3"], Float],
        NDArray[Shape["N"], Float],
    ]:
        # the processing of partial segmented pointlcouds is a mess in the original code
        # therefore we did not put it into the preprocessing pipeline and rather adapted
        # the model wrapper to handle it

        pc_segments = {-1: pc_segment} if pc_segment is not None else {}
        local_regions = pc_segment is not None
        filter_grasps = pc_segment is not None

        (
            pred_grasps_cam,
            scores,
            contact_pts,
            gripper_openings,
        ) = self._grasp_estimator.predict_scene_grasps(
            self._sess,
            pc_full,
            pc_segments=pc_segments,
            local_regions=local_regions,
            filter_grasps=local_regions,
            # forward_passes=1,
        )

        assert (
            len(pred_grasps_cam)
            == len(scores)
            == len(contact_pts)
            == len(gripper_openings)
            in (0, 1)
        )

        if len(pred_grasps_cam) == 0:
            return np.array([]), np.array([]), np.array([]), np.array([])

        pred_grasps_cam = pred_grasps_cam[-1]
        scores = scores[-1]
        contact_pts = contact_pts[-1]
        gripper_openings = gripper_openings[-1]

        # this is bug in the original code: if only one grasps gets predicted, the width output is not a array but a single float
        if gripper_openings.ndim == 0:
            gripper_openings = np.array([gripper_openings])

        return pred_grasps_cam, scores, contact_pts, gripper_openings


# from typing import List
# from pathlib import Path
# import sys

# from grasping_benchmarks.base import BaseGraspPlanner, CameraData, Grasp6D


# class ContactGraspnetPlanner(BaseGraspPlanner):
#     def __init__(self, contact_graspnet_repo_path: Path):
#         sys.path.append(str(contact_graspnet_repo_path))
#         # from contact_graspnet import ContactGraspnet

#         # self.contact_graspnet = ContactGraspnet()

#     def plan_grasp(
#         self, camera_data: CameraData, n_candidates: int = 1
#     ) -> List[Grasp6D]:
#         pass

# python grasping_benchmarks/contact_graspnet/contact_graspnet_planner.py --ckpt_dir /home/contact_graspnet/checkpoints/scene_test_2048_bs3_hor_sigma_001 --np_path /home/contact_graspnet/test_data/1.npy --local_regions --filter_grasps

# def predict_scene_grasps(
#         self,
#         sess,
#         pc_full,
#         pc_segments={},
#         local_regions=False,
#         filter_grasps=False,
#         forward_passes=1,
#     ):
#         """
#         Predict num_point grasps on a full point cloud or in local box regions around point cloud segments.

#         Arguments:
#             sess {tf.Session} -- Tensorflow Session
#             pc_full {np.ndarray} -- Nx3 full scene point cloud

#         Keyword Arguments:
#             pc_segments {dict[int, np.ndarray]} -- Dict of Mx3 segmented point clouds of objects of interest (default: {{}})
#             local_regions {bool} -- crop 3D local regions around object segments for prediction (default: {False})
#             filter_grasps {bool} -- filter grasp contacts such that they only lie within object segments (default: {False})
#             forward_passes {int} -- Number of forward passes to run on each point cloud. (default: {1})

#         Returns:
#             [np.ndarray, np.ndarray, np.ndarray, np.ndarray] -- pred_grasps_cam, scores, contact_pts, gripper_openings
#         """

import os
import sys
import argparse
import glob

import numpy as np
import tensorflow.compat.v1 as tf

sys.path.append("/home/contact_graspnet/contact_graspnet")

tf.disable_eager_execution()
physical_devices = tf.config.experimental.list_physical_devices("GPU")
tf.config.experimental.set_memory_growth(physical_devices[0], True)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(BASE_DIR))
import config_utils
from data import regularize_pc_point_count, depth2pc, load_available_input_data

from contact_grasp_estimator import GraspEstimator
from visualization_utils import visualize_grasps, show_image


class ContactGraspnetModel:
    def __init__(self, global_config, checkpoint_dir) -> None:
        self._grasp_estimator = GraspEstimator(global_config)
        self._grasp_estimator.build_network()
        saver = tf.train.Saver(save_relative_paths=True)
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        self.sess = tf.Session(config=config)
        self.grasp_estimator.load_weights(self.sess, saver, checkpoint_dir, mode="test")

        self.K = K

    def __call__(self):
        pc_segments = {}
        segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(
            p, K=K
        )

        if segmap is None and (local_regions or filter_grasps):
            raise ValueError(
                "Need segmentation map to extract local regions or filter grasps"
            )

        if pc_full is None:
            print("Converting depth to point cloud(s)...")
            pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(
                depth,
                cam_K,
                segmap=segmap,
                rgb=rgb,
                skip_border_objects=skip_border_objects,
                z_range=z_range,
            )

        print("Generating Grasps...")
        (
            pred_grasps_cam,
            scores,
            contact_pts,
            _,
        ) = self.grasp_estimator.predict_scene_grasps(
            self.sess,  # tf session
            pc_full,  # point cloud Nx3 of full scene
            pc_segments=pc_segments,  # dict of Mx3 segmented point clouds of objects of interest, keys are integer ids
            local_regions=local_regions,  # bool
            filter_grasps=filter_grasps,
            forward_passes=forward_passes,
        )


def inference(
    global_config,
    checkpoint_dir,
    input_paths,
    K=None,
    local_regions=True,
    skip_border_objects=False,
    filter_grasps=True,
    segmap_id=None,
    z_range=[0.2, 1.8],
    forward_passes=1,
):
    """
    Predict 6-DoF grasp distribution for given model and input data

    :param global_config: config.yaml from checkpoint directory
    :param checkpoint_dir: checkpoint directory
    :param input_paths: .png/.npz/.npy file paths that contain depth/pointcloud and optionally intrinsics/segmentation/rgb
    :param K: Camera Matrix with intrinsics to convert depth to point cloud
    :param local_regions: Crop 3D local regions around given segments.
    :param skip_border_objects: When extracting local_regions, ignore segments at depth map boundary.
    :param filter_grasps: Filter and assign grasp contacts according to segmap.
    :param segmap_id: only return grasps from specified segmap_id.
    :param z_range: crop point cloud at a minimum/maximum z distance from camera to filter out outlier points. Default: [0.2, 1.8] m
    :param forward_passes: Number of forward passes to run on each point cloud. Default: 1
    """

    # Build the model
    grasp_estimator = GraspEstimator(global_config)
    grasp_estimator.build_network()

    # Add ops to save and restore all the variables.
    saver = tf.train.Saver(save_relative_paths=True)

    # Create a session
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    config.allow_soft_placement = True
    sess = tf.Session(config=config)

    # Load weights
    grasp_estimator.load_weights(sess, saver, checkpoint_dir, mode="test")

    os.makedirs("results", exist_ok=True)

    # Process example test scenes
    for p in glob.glob(input_paths):
        print("Loading ", p)

        pc_segments = {}
        segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(
            p, K=K
        )

        if segmap is None and (local_regions or filter_grasps):
            raise ValueError(
                "Need segmentation map to extract local regions or filter grasps"
            )

        if pc_full is None:
            print("Converting depth to point cloud(s)...")
            pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(
                depth,
                cam_K,
                segmap=segmap,
                rgb=rgb,
                skip_border_objects=skip_border_objects,
                z_range=z_range,
            )

        print("Generating Grasps...")
        pred_grasps_cam, scores, contact_pts, _ = grasp_estimator.predict_scene_grasps(
            sess,
            pc_full,
            pc_segments=pc_segments,
            local_regions=local_regions,
            filter_grasps=filter_grasps,
            forward_passes=forward_passes,
        )

        # Save results
    #     np.savez(
    #         "results/predictions_{}".format(
    #             os.path.basename(p.replace("png", "npz").replace("npy", "npz"))
    #         ),
    #         pred_grasps_cam=pred_grasps_cam,
    #         scores=scores,
    #         contact_pts=contact_pts,
    #     )

    #     # Visualize results
    #     show_image(rgb, segmap)
    #     visualize_grasps(
    #         pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colors
    #     )

    # if not glob.glob(input_paths):
    #     print("No files found: ", input_paths)


if __name__ == "__main__":
    ckpt_dir = "/home/contact_graspnet/checkpoints/scene_test_2048_bs3_hor_sigma_001"
    np_path = "/home/contact_graspnet/test_data/7.npy"
    K = None
    z_range = [0.2, 1.8]
    local_regions = True
    filter_grasps = True
    skip_border_objects = False
    forward_passes = 1
    segmap_id = 0
    arg_configs = []

    global_config = config_utils.load_config(
        ckpt_dir, batch_size=forward_passes, arg_configs=arg_configs
    )

    inference(
        global_config,
        ckpt_dir,
        np_path,
        z_range=eval(str(z_range)),
        K=K,
        local_regions=local_regions,
        filter_grasps=filter_grasps,
        segmap_id=segmap_id,
        forward_passes=forward_passes,
        skip_border_objects=skip_border_objects,
    )
