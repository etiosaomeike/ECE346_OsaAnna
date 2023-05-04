#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from task1_traj_planner.py import TrajectoryPlanner

if __name__ == '__main__':
    rospy.init_node('Final Project Task 1 Node')
    rospy.loginfo("Start Final Project Task 1 Node")

    planner = TrajectoryPlanner()
    rospy.spin()
