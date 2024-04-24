from dataclasses import dataclass

import numpy as np

from grasping_benchmarks.base import NpArray


@dataclass
class CameraData:
    rgb_img: NpArray["H, W, 3", np.uint8] = None
    depth_img: NpArray["H, W", np.uint16] = None
    pointcloud: NpArray["N, 3", np.float32] = None
    seg_img: NpArray["H, W", np.uint8] = None
    cam_intrinsics: NpArray["3, 3", np.float32] = None
    cam_pos: NpArray["3", np.float32] = None
    cam_rot: NpArray["3, 3", np.float32] = None
