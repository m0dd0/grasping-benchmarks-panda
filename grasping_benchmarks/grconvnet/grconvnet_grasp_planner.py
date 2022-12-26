
from grconvnet_grasping.orig.load_models import load_pretrained_models

from grasping_benchmarks.base.base_grasp_planner import BaseGraspPlanner, CameraData


class GRConvNetGraspPlanner(BaseGraspPlanner):
    def __init__(self, model_name, version_name):
        cfg = {"model_name": model_name, "version_name": version_name}
        super(GRConvNetGraspPlanner, self).__init__(cfg)

        self.net = load_pretrained_models(model_name, version_name)


    def plan_grasp(self, camera_data: CameraData, n_candidates: int = 1):
        """Computes the given number of grasp candidates from from the given
        camera data.

        Args:
            camera_data (CameraData): Contains the data to compute the grasp poses
            n_candidates (int, optional): The number of grasp candidates to compute. Defaults to 1.
        """

        self._camera_data = camera_data

        

        
        self._best_grasp = 
        self._grasp_poses = 


    def visualize(self):
        """Plot the grasp poses"""
        raise NotImplementedError()

    # @property
    # def grasp_poses(self):
    #     return self._grasp_poses

    # @grasp_poses.setter
    # def grasp_poses(self, grasp_poses: List[Grasp6D]):
    #     if not all([type(p) is Grasp6D for p in grasp_poses]):
    #         raise ValueError(
    #             "Invalid grasp type. Must be `benchmark_grasping.grasp.Grasp6D`"
    #         )

    #     self._grasp_poses = grasp_poses

    # @property
    # def best_grasp(self):
    #     return self._best_grasp

    # @best_grasp.setter
    # def best_grasp(self, best_grasp: Grasp6D):
    #     if type(best_grasp) is not Grasp6D:
    #         raise ValueError(
    #             "Invalid grasp type. Must be `benchmark_grasping.grasp.Grasp6D`"
    #         )

    #     self._best_grasp = best_grasp
