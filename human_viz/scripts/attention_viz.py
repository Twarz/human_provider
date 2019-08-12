#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3, Point
from visualization_msgs.msg import Marker
from sensor_msgs import point_cloud2
from human_visual_attention.msg import HumanAttentionArray
import colorsys

class AttentionViz(object):

    def __init__(self, attention_topic_sub, body_topic_pub):
        self.debug_pub = rospy.Publisher(body_topic_pub, Marker, queue_size=1)
        self.attention_sub = rospy.Subscriber(attention_topic_sub, HumanAttentionArray, callback=self.callback)

    def callback(self, attentions):
        if not attentions.humans:
            return 
        markers = Marker(header=attentions.header, type=Marker.POINTS, action=Marker.ADD)
        markers.scale.x = 0.04
        markers.scale.y = 0.04

        for h in attentions.humans:
            if h.element_of_attention != 'nothing':
                #center = Marker(header=attentions.header, type=Marker.SPHERE, action=Marker.ADD)
                markers.points.append(h.point_of_attention)
                markers.colors.append(ColorRGBA(r=0, g=1, b=0, a=1))

            for element in h.elements:
                for p in point_cloud2.read_points(element, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
                    (r, g, b) = colorsys.hsv_to_rgb(min(1-p[3], 0.8333), 1.0, 1.0)
                    markers.points.append(Point(x=p[0], y=p[1], z=p[2]))
                    markers.colors.append(ColorRGBA(r=r, g=g, b=b, a=1))

        self.debug_pub.publish(markers)


if __name__ == '__main__':
    rospy.init_node("attention_viz_node") 
    attention_topic_sub = "/humans/visual/cumulated"
    body_topic_pub = '/humans/visual/cumulated/viz'
    AttentionViz(attention_topic_sub, body_topic_pub)
    rospy.spin()