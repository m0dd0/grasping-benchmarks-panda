from dataclasses import dataclass

import numpy as np


@dataclass
class CameraData:
    rgb_img: np.ndarray = None  # NpArray["H, W, 3", np.uint8] = None
    depth_img: np.ndarray = None  # NpArray["H, W", np.uint16] = None
    pointcloud: np.ndarray = None  # NpArray["N, 3", np.float32] = None
    seg_img: np.ndarray = None  # NpArray["H, W", np.uint8] = None
    cam_intrinsics: np.ndarray = None  # NpArray["3, 3", np.float32] = None
    cam_pos: np.ndarray = None  # NpArray["3", np.float32] = None
    cam_rot: np.ndarray = None  # NpArray["3, 3", np.float32] = None
