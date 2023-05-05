#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from task1_traj_planner import Task1_Controller

if __name__ == '__main__':
    rospy.init_node('task1_waypoint_nav_node')
    rospy.loginfo("Start Final Project Task 1 Node")
    Task1_Controller()
    rospy.spin()
