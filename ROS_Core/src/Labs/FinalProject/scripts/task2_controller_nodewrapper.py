#!/usr/bin/env python
import rospy
import sys, os
from task2_controller import Task2_Controller

# converted to same structure of task2 world node vs swifthaul file

if __name__ == '__main__':
    # Safe guard for GPU memory
    rospy.init_node('ece346_final_task2_ontroller_node')
    Task2_Controller()
    rospy.spin()
