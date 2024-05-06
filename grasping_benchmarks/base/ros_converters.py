import numpy as np
import ros_numpy

from grasping_benchmarks.base import CameraData

import rospy
from geometry_msgs.msg import PoseStamped

# the following are imports from the grasping_benchmarks_ros ROS package
from grasping_benchmarks_ros.srv import GraspPlannerRequest
from grasping_benchmarks_ros.msg import BenchmarkGrasp


def grasp_to_ros_message(grasp) -> BenchmarkGrasp:
    grasp_msg = BenchmarkGrasp()

    pose = PoseStamped()
    pose.header.frame_id = grasp.ref_frame
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = grasp.position[0]
    pose.pose.position.y = grasp.position[1]
    pose.pose.position.z = grasp.position[2]
    pose.pose.orientation.x = grasp.quaternion[0]
    pose.pose.orientation.y = grasp.quaternion[1]
    pose.pose.orientation.z = grasp.quaternion[2]
    pose.pose.orientation.w = grasp.quaternion[3]
    grasp_msg.pose = pose

    grasp_msg.score.data = grasp.score
    grasp_msg.width.data = grasp.width

    return grasp_msg


def service_request_to_camera_data(req: GraspPlannerRequest):
    camera_trafo_h = ros_numpy.numpify(req.view_point.pose)

    camera_data = CameraData(
        rgb_image=ros_numpy.numpify(req.color_image),
        depth_image=ros_numpy.numpify(req.depth_image),
        pointcloud=ros_numpy.numpify(req.cloud).view(np.float32).reshape(-1, 3).copy(),
        pointcloud_segmented=ros_numpy.numpify(req.pointcloud_segmented).view(np.float32).reshape(-1, 3).copy(),
        segmentation_image=ros_numpy.numpify(req.seg_image),
        camera_matrix=np.array(req.camera_info.K).reshape(3, 3),
        camera_trafo_h[:3, 3],
        camera_trafo_h[:3, :3],
    )
