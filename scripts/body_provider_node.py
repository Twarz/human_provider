#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image, CameraInfo
#from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

from jsk_recognition_msgs.msg import PeoplePoseArray, PeoplePose

from tfpose_ros.msg import Persons
from tf_pose.common import CocoPart

from human_provider.body_estimator3D import BodyEstimator3D

import numpy as np

MAP_ID_TO_NAME = {}
for p in CocoPart:
    MAP_ID_TO_NAME[p.value] = p.name

class BodyProvider(object):

    def __init__(self, camera_info_topic, camera_depth_topic):
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo)
        self.body_estimator3D = BodyEstimator3D(camera_info)
        self.bridge = CvBridge()
        self.pose3D_pub = rospy.Publisher('/pose3D', PeoplePoseArray, queue_size=2)
        self.poses_sub = message_filters.Subscriber('/pose', Persons)
        self.depth_sub = message_filters.Subscriber(camera_depth_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.poses_sub, self.depth_sub], 20, 0.1)
        self.ts.registerCallback(self.callback)
        

    def callback(self, persons, depth_img):

        # if no persons detected, just reset and pass
        if not persons.persons:
            self.body_estimator3D.reset()
            return

        timestamp = persons.header.stamp # we choose 'persons' because it's from rgb and not depth (more used)

        # get depth image as numpy array
        np_depth_img = self.bridge.imgmsg_to_cv2(depth_img, "passthrough")

        # estimate the 3D poses
        bodies2D = self.convert_persons_to_numpy_dict_list(persons.persons)
        tracked_bodies3D = self.body_estimator3D.estimate(bodies2D, np_depth_img)
        
        # publish poses
        self.publish(tracked_bodies3D, timestamp)
    
    def convert_persons_to_numpy_dict_list(self, persons):
        numpy_dict_list = []
        for p in persons:
            numpy_dict = {}
            for bp in p.body_part:
                numpy_dict[bp.part_id] = np.array([bp.x, bp.y, bp.confidence])
            numpy_dict_list.append(numpy_dict)
        return numpy_dict_list

    def publish(self, tracked_bodies3D, timestamp):
        people_pose_array = PeoplePoseArray()
        people_pose_array.header.frame_id = "head_mount_kinect2_rgb_optical_frame"
        people_pose_array.header.stamp = timestamp
        for body_id, body in tracked_bodies3D.items():
            people_pose = PeoplePose()
            for part_id, part in body.body.items():
                point = Point(x=part[0][0], y=part[0][1], z=part[0][2])
                people_pose.limb_names.append(str(body_id) + ':' + str(part_id))
                people_pose.scores.append(part[1])
                people_pose.poses.append(Pose(position=point))
            people_pose_array.poses.append(people_pose)
        
        self.pose3D_pub.publish(people_pose_array)


if __name__ == '__main__':
    rospy.init_node("body_provider_node") 
    camera_info_topic = "/head_mount_kinect2/qhd/camera_info" # "/kinect2/qhd/camera_info"
    camera_depth_topic = '/head_mount_kinect2/qhd/image_depth_rect' # '/kinect2/qhd/image_depth_rect'
    BodyProvider(camera_info_topic, camera_depth_topic)
    rospy.spin()