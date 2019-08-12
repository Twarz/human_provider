#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs import point_cloud2
from human_visual_attention.msg import HumanMemoryArray
import colorsys

class AttentionViz(object):

    def __init__(self, attention_topic_sub, viz_topic_pub):
        self.viz_pub = rospy.Publisher(viz_topic_pub, MarkerArray, queue_size=1)
        self.attention_sub = rospy.Subscriber(attention_topic_sub, HumanMemoryArray, callback=self.callback)

    def callback(self, memory_array):
        if not memory_array.humans:
            return

        markers = []
        for h in memory_array.humans:
            for element in h.elements:
                marker = Marker(header=memory_array.header, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.text = 'max:' + str(element.max_attention_value) + '\n'
                marker.text += 'count:' + str(element.max_attention_count) + '\n'
                marker.text += 'last_time:' + str(element.last_time) + '\n'
                marker.text += 'time_from_last_time:' + str(element.time_from_last_time) + '\n'
                marker.text += 'last_watchtime:' + str(element.last_watchtime)
                marker.pose.position.x = 1.0
                marker.pose.position.y = 1.0
                marker.pose.position.z = 1.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                markers.append(marker)
            
            '''
            if h.element_of_attention != 'nothing':
                #center = Marker(header=attentions.header, type=Marker.SPHERE, action=Marker.ADD)
                markers.points.append(h.point_of_attention)
                markers.colors.append(ColorRGBA(r=0, g=1, b=0, a=1))

            for element in h.elements:
                for p in point_cloud2.read_points(element, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
                    (r, g, b) = colorsys.hsv_to_rgb(min(1-p[3], 0.8333), 1.0, 1.0)
                    markers.points.append(Point(x=p[0], y=p[1], z=p[2]))
                    markers.colors.append(ColorRGBA(r=r, g=g, b=b, a=1))
            '''
        marker_array = MarkerArray(markers=markers)
        self.viz_pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node("memory_viz_node")
    attention_topic_sub = "/humans/visual/memory"
    viz_topic_pub = '/humans/visual/memory/viz'
    AttentionViz(attention_topic_sub, viz_topic_pub)
    rospy.spin()