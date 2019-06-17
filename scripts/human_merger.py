#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
"""

import rospy
import message_filters

import numpy as np

from geometry_msgs.msg import Pose, PoseStamped
from jsk_recognition_msgs.msg import PeoplePoseArray, PeoplePose
from tf import TransformListener

class HumanMerger(object):

    def __init__(self, human_gaze_pose_topic, human_body_pose_topic, human_full_pose_topic):
        self.human_pose_pub = rospy.Publisher(human_full_pose_topic, PeoplePoseArray, queue_size=5)
        self.human_body_pose_sub = message_filters.Subscriber(human_body_pose_topic, PeoplePoseArray)
        self.human_gaze_pose_sub = message_filters.Subscriber(human_gaze_pose_topic, PeoplePoseArray)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.human_body_pose_sub, self.human_gaze_pose_sub], 40, 0.3)
        self.ts.registerCallback(self.callback)
        self.tf = TransformListener()

    def callback(self, bodies, gazes):
        print('callback')
        print(bodies.header.frame_id)
        print(gazes.header.frame_id)
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
                    gaze_pose = self.find_nearest_gaze_pose(nose_pose, people_gaze)
                    
            if nose_pose and gaze_pose:
                person.limb_names.append(person_id+":gaze")
                gaze_pose = self.tf.transformPose(bodies.header.frame_id, PoseStamped(header=gazes.header, pose=gaze_pose))
                gaze_pose = gaze_pose.pose
                gaze_pose.position = nose_pose.position
                person.poses.append(gaze_pose)
                #person.scores.append(1.0)
        self.human_pose_pub.publish(full_poses)

    def compute_default_gaze_pose(self, person):
        return None

    def find_nearest_gaze_pose(self, nose_pose, people_gaze):
        print
        nearest_gaze_pose = None
        min_distance = 10e10

        for person_gaze in people_gaze:
            for gaze_pose in person_gaze.poses:
                np_gaze = np.array([gaze_pose.position.x, gaze_pose.position.y, gaze_pose.position.z])
                np_nose = np.array([nose_pose.position.x, nose_pose.position.y, nose_pose.position.z])
                d = np.sqrt(np.sum(np.square(np_gaze-np_nose)))
                print(d)
                if d < 4. and d < min_distance:
                    nearest_gaze_pose = gaze_pose
                    min_distance = d
        return nearest_gaze_pose

if __name__ == '__main__':
    rospy.init_node("human_merger") 
    human_gaze_pose_topic = '/humans/poses/3D/gaze'
    human_body_pose_topic = '/humans/poses/3D/body'
    human_full_pose_topic = '/humans/poses/3D/full'
    HumanMerger(human_gaze_pose_topic, human_body_pose_topic, human_full_pose_topic)
    print('ready+')
    rospy.spin()