#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
"""

import rospy
import message_filters

import numpy as np

from geometry_msgs.msg import Pose
from jsk_recognition_msgs.msg import PeoplePoseArray, PeoplePose

class HumanMerger(object):

    def __init__(self, human_gaze_pose_topic, human_body_pose_topic, human_full_pose_topic):
        self.human_pose_pub = rospy.Publisher(human_full_pose_topic, PeoplePoseArray, queue_size=2)
        self.human_body_pose_sub = message_filters.Subscriber(human_body_pose_topic, PeoplePoseArray)
        self.human_gaze_pose_sub = message_filters.Subscriber(human_gaze_pose_topic, PeoplePoseArray)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.human_body_pose_sub, self.human_gaze_pose_sub], 20, 0.1)
        self.ts.registerCallback(self.callback)

    def callback(self, bodies, gazes):
        full_poses = bodies # same message
        people_gaze = gazes.poses
       
        for person in full_poses.poses:
            person_id = "0"
            gaze_pose = None # self.compute_default_gaze_pose(person)
            nose_pose = None
            for limb_name, limb_pose in zip(person.limb_names, person.poses):
                person_id, limb_id = limb_name.split(':')
                if limb_id == "0": # noze, only once
                    nose_pose = limb_pose
                    gaze_pose = self.find_nearest_gaze_pose(people_gaze)
            if gaze_pose and nose_pose:
                person.limb_names.append(person_id+":gaze")
                gaze_pose.position = nose_pose.position
                person.poses.append(gaze_pose)
                person.scores.append(1.0)
        self.human_pose_pub.publish(full_poses)

    def compute_default_gaze_pose(self, person):
        return None

    def find_nearest_gaze_pose(self, nose_pose, people_gaze):
        nearest_gaze_pose = None
        min_distance = 10e10

        for person_gaze in people_gaze:
            for gaze_name, gaze_pose in zip(person_gaze.limb_names, person_gaze.poses):
                d = np.sqrt(np.sum(np.square(gaze_pose.position-nose_pose.position)))
                if d < 0.25 and d < min_distance:
                    nearest_gaze_pose = gaze_pose
                    min_distance = d
        return nearest_gaze_pose

if __name__ == '__main__':
    rospy.init_node("human_merger") 
    human_gaze_pose_topic = '/humans/poses/3D/gaze'
    human_body_pose_topic = '/humans/poses/3D/body'
    human_full_pose_topic = '/humans/poses/3D/full'
    HumanMerger(people_gaze_pose_topic, people_body_pose_topic, human_full_pose_topic)
    rospy.spin()