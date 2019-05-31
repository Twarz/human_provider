#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
"""

import rospy
import message_filters

from jsk_recognition_msgs.msg import PeoplePoseArray, PeoplePose

class HumanProvider(object):

    def __init__(self, people_gaze_pose_topic, people_body_pose_topic):
        self.human_pose_pub = rospy.Publisher('/humans/poses/3D/full', PeoplePoseArray, queue_size=2)
        self.human_body_pose_sub = message_filters.Subscriber('/humans/poses/3D/body', PeoplePoseArray)
        self.human_gaze_pose_sub = message_filters.Subscriber('/humans/poses/3D/gaze', PeoplePoseArray)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.human_body_pose_sub, self.human_gaze_pose_sub], 20, 0.1)
        self.ts.registerCallback(self.callback)

    def callback(self, bodies, gazes):
        full_poses = PeoplePoseArray()
        full_poses.header.stamp = bodies.header.stamp
        full_poses.header.frame_id = bodies.header.frame_id
        


        self.human_pose_pub.publish(full_poses)