#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from jsk_recognition_msgs.msg import PeoplePoseArray
from pyuwds.types.gen_uuid import gen_uuid
from pyuwds.uwds_client import UwdsClient
from pyuwds.uwds import PROVIDER
from uwds_msgs.msg import Node, Changes, Property
from pyuwds.types.nodes import CAMERA, ENTITY
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance

CAMERA_PROPERTIES = []
CAMERA_PROPERTIES.append(Property(name="hfov",     data="60.0"))
CAMERA_PROPERTIES.append(Property(name="aspect",   data="1.3333"))
CAMERA_PROPERTIES.append(Property(name="clipnear", data="0.3"))
CAMERA_PROPERTIES.append(Property(name="clipfar",  data="100.0"))
CAMERA_PROPERTIES.append(Property(name="class",    data="Eyes"))

class UwdsNodeBridge(UwdsClient):
    def __init__(self, ros_topic, uwds_world, uwds_client_name, ros_msg_type):
        super(UwdsNodeBridge, self).__init__(uwds_client_name, PROVIDER)
        self.uwds_nodes = {}
        self.ros_to_uwds_id = {}
        self.output_world = uwds_world # override 'output_world' attribute
        self.ros_topic_sub = rospy.Subscriber(ros_topic, ros_msg_type, self.update, queue_size=10)   

    def update(self, ros_msg):
        current_nodes = self.ros_to_uwds_id.keys()
        print(current_nodes)
        print(self.uwds_nodes.keys())
        nodes_to_update_ids = self.get_nodes_to_update(ros_msg, current_nodes)
        nodes_to_delete_ids = []
        if not nodes_to_update_ids:
            if self.uwds_nodes:
                nodes_to_delete_ids = self.ros_to_uwds_id.keys()
            else:
                return
        changes = Changes()
        header = Header(stamp=ros_msg.header.stamp, frame_id=ros_msg.header.frame_id)
        
        # delete old nodes
        for i in list(set(current_nodes) - set(nodes_to_update_ids)):
            nodes_to_delete_ids.append(i)
        
        for node_id in nodes_to_update_ids:
            changes.nodes_to_update.append(self.get_node(node_id))
        
        for node_id in nodes_to_delete_ids:
            changes.nodes_to_delete.append(node_id)
            self.remove_node(node_id)

        print('submitting....')
        self.submit_changes(changes, header)
        print('ok')

    def add_node(self, node, ros_id):
        self.uwds_nodes[node.id] = node
        self.ros_to_uwds_id[ros_id] = node.id

    def remove_node(self, node_ros_id):
        del self.uwds_nodes[self.ros_to_uwds_id[node_ros_id]]
        del self.ros_to_uwds_id[node_ros_id]

    def submit_changes(self, changes, header):
        self.ctx.worlds()[self.output_world].update(changes, header=header)

    def get_node(self, ros_id):
        return self.uwds_nodes[self.ros_to_uwds_id[ros_id]]
    
    def get_nodes_to_update(self, ros_msg, current_nodes):
        raise NotImplementedError()

    

class UwdsHumanBridge(UwdsNodeBridge):
    def __init__(self, ros_topic, uwds_world):
        super(UwdsHumanBridge, self).__init__(ros_topic, uwds_world, 'uwds_human_bridge', PeoplePoseArray)

    def add_head_node(self, pose, ros_id):
        node = Node(id=gen_uuid(), name="head", type=CAMERA, properties=CAMERA_PROPERTIES, position=pose)
        self.add_node(node, ros_id)

    def add_body_node(self, pose, ros_id):
        node = Node(id=gen_uuid(), name=ros_id, type=ENTITY, position=pose)
        self.add_node(node, ros_id)

    def get_nodes_to_update(self, ros_human_msg, current_nodes):
        # if no new human 
        if not ros_human_msg.poses:
            return []
        people = ros_human_msg.poses
        nodes_to_update = []
        for person in people:
            for limb_name, limb_pose in zip(person.limb_names, person.poses):
                if limb_name in current_nodes:
                    self.update_node_position(limb_name, limb_pose)
                else:
                    if 'gaze' in limb_name:
                        self.add_head_node(PoseWithCovariance(pose=limb_pose), limb_name)
                    else:
                        self.add_body_node(PoseWithCovariance(pose=limb_pose), limb_name)
                nodes_to_update.append(limb_name)
        return nodes_to_update

    def update_node_position(self, node_ros_id, node_new_position):
        print type(node_new_position)
        print type(self.get_node(node_ros_id))
        self.get_node(node_ros_id).position.pose = node_new_position


if __name__ == '__main__':
    rospy.init_node("uwds_human_bridge") 
    ros_topic = "/humans/poses/3D/gaze"
    uwds_world = '/env/human'
    UwdsHumanBridge(ros_topic, uwds_world)
    rospy.spin()