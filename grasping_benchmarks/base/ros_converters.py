import numpy as np
import ros_numpy

from grasping_benchmarks.base import CameraData

import rospy
from geometry_msgs.msg import PoseStamped
from grasping_benchmarks.ros.srv import GraspPlannerRequest
from grasping_benchmarks.ros.msg import BenchmarkGrasp


def grasp_data_to_service_response(self) -> BenchmarkGrasp:
    grasp_msg = BenchmarkGrasp()

    p = PoseStamped()
    p.header.frame_id = self.ref_frame
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = self.position[0]
    p.pose.position.y = self.position[1]
    p.pose.position.z = self.position[2]
    p.pose.orientation.x = self.quaternion[0]
    p.pose.orientation.y = self.quaternion[1]
    p.pose.orientation.z = self.quaternion[2]
    p.pose.orientation.w = self.quaternion[3]
    grasp_msg.pose = p

    grasp_msg.score.data = self.score
    grasp_msg.width.data = self.width

    return grasp_msg


def service_request_to_camera_data(req: GraspPlannerRequest):
    rgb = ros_numpy.numpify(req.color_image)
    depth = ros_numpy.numpify(req.depth_image)
    seg = ros_numpy.numpify(req.seg_image)

    pc = ros_numpy.numpify(req.cloud)

    camera_matrix = np.array(req.camera_info.K).reshape(3, 3)

    # 4x4 homogenous tranformation matrix
    camera_trafo_h = ros_numpy.numpify(req.view_point.pose)

    camera_data = CameraData(
        rgb,
        depth,
        pc,  # TODO check format and convert if needed
        seg,
        camera_matrix,
        camera_trafo_h[:3, 3],
        camera_trafo_h[:3, :3],
    )
