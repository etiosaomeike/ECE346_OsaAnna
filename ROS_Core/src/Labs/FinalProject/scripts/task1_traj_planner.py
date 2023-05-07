#!/usr/bin/env python
import rospy
import numpy as np
from task2_world.util import get_ros_param
from racecar_routing.srv import Plan, PlanResponse, PlanRequest
from task2_world.util import RefPath
import yaml
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
import threading

# Pull in all needed info/services as descirbed in readme file

class Task1_Controller:
    def __init__(self):
        
        print("The Task1 Controller Has Been Initialized")
        self.truck_x = 0
        self.truck_y = 0
        self.goal_positions, self.goal_order = self.parse_yaml("/Users/jeremiahomeike/Desktop/ECE346_OsaAnna/ECE346_OsaAnna/ROS_Core/src/Labs/FinalProject/task1.yaml")
        self.path_topic = get_ros_param('~path_topic', '/Routing/Path')
        self.odom_topic = get_ros_param('~odom_topic', '/Simulation/Pose')
        self.setup_clients()
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)

        threading.Thread(target=self.loop).start()

    def odometry_callback(self, odom_msg):
        self.truck_x = odom_msg.pose.pose.position.x
        self.truck_y = odom_msg.pose.pose.position.y
  
    def parse_yaml(self, yaml_path):
        list_of_goals = []
        with open(yaml_path, 'r') as f:
            doc = yaml.load(f, Loader=yaml.FullLoader)

        for idx in range(1,13):
            list_of_goals.append(doc["goal_{}".format(idx)])
        
        indexes_of_goals = doc["goal_order"]
        indexes_of_goals = np.array(indexes_of_goals)
        indexes_of_goals -= 1 # shift the indexing by one 
        indexes_of_goals = list(indexes_of_goals)

        return list_of_goals, indexes_of_goals

    def setup_clients(self):
        rospy.wait_for_service('/routing/plan')
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)
        self.path_pub = rospy.Publisher(self.path_topic, PathMsg)

    def pub_ref_path(self, start, goal):
        x_start = start[0]
        y_start = start[1]

        x_goal = goal[0]
        y_goal = goal[1]

        plan_request = PlanRequest([x_start, y_start], [x_goal, y_goal])
        plan_response = self.plan_client(plan_request)
        path = plan_response.path
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'map'
        self.path_pub.publish(path)
    
    def loop(self):
        pointer = 0
        threshold = .8
        x = self.truck_x
        y = self.truck_y
        state = np.array([x, y])
        norm = np.linalg.norm(state - self.goal_positions[self.goal_order[pointer]])
        
        while rospy.is_shutdown() is False:
       ##     if pointer == 0:   
               # for idx in range(len(self.list_of_goals))      
               #   
            x = self.truck_x
            y = self.truck_y
            state = np.array([x, y])

            norm = np.linalg.norm(state - self.goal_positions[self.goal_order[pointer]])
            if (pointer == 0) or (pointer == 1):
                self.pub_ref_path(state, self.goal_positions[self.goal_order[pointer]])
                print("We published the path becuase the pointer is currently 0 or 1!!!")

            if norm < threshold:
                pointer += 1
                self.pub_ref_path(state, self.goal_positions[self.goal_order[pointer]])
                print("Print we're at a new pointer now")
                
            else:
                pass
               # rospy.sleep(.6)

            if pointer >= len(self.goal_order) - 1:
                break

    






