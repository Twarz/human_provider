#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from pyuwds.types.gen_uuid import gen_uuid
from pyuwds.uwds import UnderworldsProxy, PROVIDER
from uwds_msgs.msg import Node, Changes, Property
from pyuwds.types.nodes import CAMERA
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Point, PoseWithCovariance
from rt_gene.msg import NamedPoses
import numpy as np
import tf

class HeadPosesProvider():
    """
    """
    def __init__(self):
    	self.__ctx = UnderworldsProxy("head_poses_provider", PROVIDER)
        self.__world = self.__ctx.worlds()['robot/gazes']
        self.nodes = {}
        self.id_to_node_id = {}
        self.properties = []
        self.properties.append(Property(name="hfov",     data="60.0"))
        self.properties.append(Property(name="aspect",   data="1.3333"))
        self.properties.append(Property(name="clipnear", data="0.3"))
        self.properties.append(Property(name="clipfar",  data="100.0"))
        self.properties.append(Property(name="class",    data="Eyes"))
        self.head_poses_sub = rospy.Subscriber('subjects/head_poses', NamedPoses, self.callback, queue_size=1)

    def callback(self, named_poses):
        changes = Changes()

        if len(named_poses.poses) > 0:
            current_nodes = self.id_to_node_id.keys()
            updated_nodes = []
            for p_id, p in zip(named_poses.pose_ids, named_poses.poses):
                if p_id in self.id_to_node_id.keys():
                    head_node = self.nodes[self.id_to_node_id[p_id]]
                    head_node.position.pose = p
                    updated_nodes.append(p_id)
                else:
                    head_node = self.create_node("head", CAMERA, parent='0')
                    head_node.position.pose = p
                    self.id_to_node_id[p_id] = head_node.id
                    self.nodes[head_node.id] = head_node
                changes.nodes_to_update.append(head_node)

            # delete old nodes
            for i in list(set(current_nodes) - set(updated_nodes)):
                changes.nodes_to_delete.append(self.id_to_node_id[i])
                del self.nodes[self.id_to_node_id[i]]
                del self.id_to_node_id[i]
            
        else:
            changes.nodes_to_delete = self.nodes.keys()
            self.nodes = {}
            self.id_to_node_id = {}

        if len(changes.nodes_to_delete) > 0 or len(changes.nodes_to_update) > 0:
            h = Header()
            h.stamp = named_poses.header.stamp
            h.frame_id = 'head_mount_kinect2_rgb_optical_frame'
            self.__ctx.worlds()['robot/gazes'].update(changes, header=h)

    """[summary]
    """
    def create_node(self, name, type, parent=''):
        return Node(id=gen_uuid(), name=name, type=type, parent=parent, properties=self.properties)

    """[summary]
    """
    def clear_world(self):
        print 'world cleared!'


if __name__ == '__main__':
    rospy.init_node("head_poses_provider", disable_signals=True)
    provider = HeadPosesProvider()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        provider.clear_world()
        print 'shutting down is 5s...'
        rospy.sleep(5)
        rospy.signal_shutdown('')

    
    
