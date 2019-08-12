#!/usr/bin/env python

"""
@Kevin Cortacero <cortacero.k31130@gmail.com>
"""

import rospy
from human_visual_attention.msg import HumanMemoryArray

class ArmExperiment(object):

    def __init__(self, memory_topic_sub):
        #TODO
        self.memory_sub = rospy.Subscriber(memory_topic_sub, HumanMemoryArray, callback=self.callback)

    def is_looking_arm(self, humans):
        for h in humans:
            for element in h.elements:
                if element.element_id == "right_arm":
                    if element.time_from_last_time == 0.0:
                        return True
        return False

    def move_arm(self):
        pass

    def stop_arm(self):
        pass

    def callback(self, memory_array):
        if not memory_array.humans:
            return
        
        if self.is_looking_arm(memory_array.humans):
            self.move_arm()
        else:
            self.stop_arm()

if __name__ == '__main__':
    rospy.init_node("PR2_arm_experiment")
    memory_topic_sub = "/humans/visual/memory"
    ArmExperiment(attention_topic_sub)
    rospy.spin()