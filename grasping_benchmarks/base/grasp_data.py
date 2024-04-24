from dataclasses import dataclass

import numpy as np


@dataclass
class Grasp:
    position: np.ndarray  # NpArray["3", np.float32] = np.zeros(3)
    rotation: np.ndarray  # NpArray["3, 3", np.float32] = np.eye(3)
    width: float = 0.0
    score: float = 0.0
    ref_frame: str = "camera"
