from dataclasses import dataclass

from nptyping import NDArray, Shape, Float, Int


@dataclass
class Grasp6D:
    position: NDArray[Shape["3"], Float]
    rotation: NDArray[Shape["3, 3"], Float]
    width: float = 0.0
    score: float = 0.0
    ref_frame: str = "camera"


@dataclass
class CameraData:
    rgb_img: NDArray[Shape["H, W, 3"], Int] = None
    depth_img: NDArray[Shape["H, W"], Int] = None
    pointcloud: NDArray[Shape["N, 3"], Float] = None
    seg_img: NDArray[Shape["H, W"], Int] = None
    cam_intrinsics: NDArray[Shape["3, 3"], Float] = None
    cam_pos: NDArray[Shape["3"], Float] = None
    cam_rot: NDArray[Shape["3, 3"], Float] = None

    def __post_init__(self):
        # TODO input validation