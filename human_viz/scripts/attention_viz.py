#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3, Point
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs import point_cloud2
from human_visual_attention.msg import HumanAttentionArray

class AttentionViz(object):

    def __init__(self, attention_topic_sub, body_topic_pub):
        self.debug_pub = rospy.Publisher(body_topic_pub, MarkerArray, queue_size=10)
        self.attention_sub = rospy.Subscriber(attention_topic_sub, HumanAttentionArray, callback=self.callback)

    def callback(self, attentions):
        if not attentions.humans:
            return 
        markers = []

        for h in attentions.humans:
            if h.element_of_attention != 'nothing':
                center = Marker(header=attentions.header, type=Marker.SPHERE, action=Marker.ADD)
                center.pose.position = h.point_of_attention
                center.id = len(markers)
                center.color.r = 0.0
                center.color.g = 1.0
                center.color.b = 0.0
                center.color.a = 1.0
                center.scale.x = 0.10
                center.scale.y = 0.10
                center.scale.z = 0.10
                center.lifetime = rospy.Duration(0.5)
                markers.append(center)

            for element in h.elements:
                for p in point_cloud2.read_points(element, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
                    m = Marker(header=attentions.header)
                    m.pose.position.x = p[0]
                    m.pose.position.y = p[1]
                    m.pose.position.z = p[2]
                    m.pose.orientation.w = 1.0
                    m.type = Marker.SPHERE
                    m.action = Marker.ADD
                    m.color.r = p[3]
                    m.color.b = 1.0 - p[3]
                    m.color.g = 1.0 - m.color.r - m.color.b
                    m.lifetime = rospy.Duration(0.5)
                    m.color.a = 1.0
                    m.scale.x = 0.05
                    m.scale.y = 0.05
                    m.scale.z = 0.05
                    m.id = len(markers)
                    markers.append(m)
        
        self.debug_pub.publish(MarkerArray(markers=markers))


if __name__ == '__main__':
    rospy.init_node("attention_viz_node") 
    attention_topic_sub = "/humans/visual_attention"
    body_topic_pub = '/humans/visual_attention/markers'
    AttentionViz(attention_topic_sub, body_topic_pub)
    rospy.spin()