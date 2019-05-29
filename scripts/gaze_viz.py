#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3, Point
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import MarkerArray, Marker

from jsk_recognition_msgs.msg import PeoplePoseArray

class GazeViz(object):

    def __init__(self, gaze_topic_sub, gaze_topic_pub):
        self.debug_pub = rospy.Publisher(gaze_topic_pub, MarkerArray, queue_size=10)
        self.poses_sub = rospy.Subscriber(gaze_topic_sub, PeoplePoseArray, callback=self.callback)

    def callback(self, people_pose_array):
        if not people_pose_array.poses:
            return 

        markers = []
        for person in people_pose_array.poses:
            for limb_name, limb_pose in zip(person.limb_names, person.poses):
                person_id, limb_id = limb_name.split(':')
                unique_id = int(person_id)*100 + int(20)
                
                m = Marker(pose=limb_pose, id=unique_id, type=Marker.ARROW, action=Marker.ADD, scale=Vector3(x=0.2, y=0.02, z=0.02), color=ColorRGBA(r=1, a=1.0), lifetime=rospy.Time(1))
                m.header.frame_id = people_pose_array.header.frame_id
                m.header.stamp = people_pose_array.header.stamp
                markers.append(m)
        
        self.debug_pub.publish(MarkerArray(markers=markers))


if __name__ == '__main__':
    rospy.init_node("body_viz_node") 
    gaze_topic_sub = "/poses/head"
    gaze_topic_pub = '/pose3D_markers'
    GazeViz(gaze_topic_sub, gaze_topic_pub)
    rospy.spin()