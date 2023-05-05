#!/usr/bin/env python
import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import yaml
from task2_world.util import get_ros_param
from final_project.srv import Schedule, ScheduleRequest, ScheduleResponse
from final_project.srv import Task, TaskRequest, TaskResponse
from final_project.srv import Reward, RewardRequest, RewardResponse

from geometry_msgs.msg import PoseStamped, Pose, Point

from racecar_routing.srv import Plan, PlanResponse, PlanRequest
#from You_Need_to_Define_the_import_Path_Here import RefPath

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg

import threading
import time

# Pull in all needed info/services as descirbed in readme file

class Task2_Controller:
    def __init__(self):
        self.truck_x = 0
        self.truck_y = 0
        self.boss_x = 0
        self.boss_y = 0



        ##### Get Warehouse Info
        self.warehouse_yaml = get_ros_param("/task2_world_node/warehouse_yaml", None)
        with open(self.warehouse_yaml, "r") as stream:
            self.warehouse_info = yaml.safe_load(stream)
        #print("WAREHOUSE INFO",warehouse_info)
        #output:
        # {'warehouse_A': {'id': 0, 'location': [3, 0.15], 'dxdy': [0.5, 0.25], 'probability': [0.3, 0.05, 0.15, 0.3, 0.2]}, 
        # 'warehouse_B': {'id': 1, 'location': [5.9, 3.5], 'dxdy': [0.25, 0.5], 'probability': [0.1, 0.4, 0.2, 0.2, 0.1]}, 
        # 'warehouse_C': {'id': 2, 'location': [0.15, 3.5], 'dxdy': [0.25, 0.5], 'probability': [0.5, 0.3, 0.1, 0.05, 0.05]}, 
        # 'warehouse_D': {'id': 3, 'location': [3, 2.2], 'dxdy': [0.5, 0.5], 'probability': [0.2, 0.2, 0.3, 0.1, 0.2]}, 
        # 'warehouse_E': {'id': 4, 'location': [3, 1.1], 'dxdy': [0.5, 0.25], 'probability': [0.2, 0.1, 0.3, 0.25, 0.15]}}

        #We consider the truck is in the warehouse when it is 
        # less than 0.5 meters away in longitudinal direction and 
        # 0.25 meters away in lateral direction.


        ##### Get Boss Schedule
        rospy.wait_for_service('/SwiftHaul/BossSchedule')
        self.boss_sched_client = rospy.ServiceProxy('/SwiftHaul/BossSchedule', Schedule)
        # how to call the service:
        # respond = boss_sched_client(ScheduleRequest())
        # The respond of this Schedule service has four fields:
        # respond.start_warehouse # List[int], the index of the warehouse that the Boss starts at
        # respond.goal_warehouse # List[int], the index of the warehouse that the Boss heading to
        # respond.reward # List[float], the reward of Boss Tasks
        # respond.schedule # List[float], the time that the Boss will depature for the task


        ##### Get Currently Available Side Task
        # set up the service client
        rospy.wait_for_service('/SwiftHaul/SideTask')
        self.sidetask_client = rospy.ServiceProxy('/SwiftHaul/SideTask', Task)

        # how to call the service
        # respond = sidetask_client(TaskRequest())
        # The respond of this Task service has three fields: (Idk it says three but there are two fields -AL)
        # respond.task # int, the index of the goal warehouse for your task. If the task is -1, it means there is no available task
        # respond.reward # float, the reward of the current tasks

        # If you want to complete the assigned side task, you can start plan immediately 
        # and drive as fast as possible for the side task. You can use the "/routing/plan" 
        # service to plan a reference path from your current position to the goal warehouse.

        # If you do not like the current available task, you can request a new task by 
        # calling the service again. The Side Task will be refreshed every 5 seconds. 
        # After refreshing, the previous task will be gone.



        #Get Currently AVailable Boss Task

        # set up the service client
        rospy.wait_for_service('/SwiftHaul/BossTask')
        self.bosstask_client = rospy.ServiceProxy('/SwiftHaul/BossTask', Task)

        # how to call the service
        # respond = bosstask_client(TaskRequest())
        # The respond of this Task service is same as the Side Task service above.
        # If you decide to take the task, you can start to plan your path to the goal 
        # warehouse. You can use the "/routing/plan" service to plan a reference path 
        # from your current position to the goal warehouse.
        # The Boss Task is only available within the 5 seconds window after the Boss 
        # departs from the warehouse. If you don't take the task within the 5 seconds window, 
        # the task will be gone.
        # Your truck have to follow the boss truck (no overtaking) if you decide to do the Boss Task. 
        # Otherwise, you session will be considered invalid.
        # You can abandon the Boss Task at any time, by requesting a new Side Task at any warehouse. 
        # You may do another Boss Task next time when the Boss departs from the warehouse


        ##### Complete the Task and Collect the Reward

        # set up the service client
        rospy.wait_for_service('/SwiftHaul/GetReward')
        self.reward_client = rospy.ServiceProxy('/SwiftHaul/GetReward', Reward)

        # how to call the service
        # respond = reward_client(RewardRequest())
        # call AFTER completion to get reward
        # The respond of this Reward service has two fields:
        # respond.done # bool, the status of the task. If the task is done, it will be True
        # respond.total_reward # float, the total reward of so far

        rospy.wait_for_service('/routing/plan')
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)


        self.odom_topic = get_ros_param('~odom_topic', '/Simulation/Pose')
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)
        
        self.boss_pose_sub = rospy.Subscriber("/Boss/Pose", Odometry, self.boss_pose_callback, queue_size=10)
        



        # set up publisher for trajectory received from routing
        self.path_topic = get_ros_param('~path_topic', '/Routing/Path')
        self.path_pub = rospy.Publisher(self.path_topic, PathMsg, queue_size=1)

        for i in range(48):
            time.sleep(1)
            print("timeout:",i)
            if rospy.is_shutdown(): break

        threading.Thread(target=self.loop).start()


        ##### Start swifthaul via call to service
        rospy.wait_for_service('/SwiftHaul/Start')
        self.start_client = rospy.ServiceProxy('/SwiftHaul/Start', Empty)
        # call the service
        self.start_client(EmptyRequest())


        print("Set up task 2 node successfully")
    
    def odometry_callback(self, odom_msg):
        self.truck_x = odom_msg.pose.pose.position.x
        self.truck_y = odom_msg.pose.pose.position.y

    def boss_pose_callback(self, odom_msg):
        self.boss_x = odom_msg.pose.pose.position.x
        self.boss_y = odom_msg.pose.pose.position.y



    def loop(self):
        warehouses = list(self.warehouse_info.keys())
        print("warehouses: ", warehouses)
        idx = 0
        self.go_to(* self.warehouse_info[warehouses[idx]]["location"])
        time.sleep(2)
        last_warehouse=idx
        times = np.zeros((len(warehouses), len(warehouses)))
        while rospy.is_shutdown() is False:  
            for i in range(len(warehouses)):
                if(rospy.is_shutdown()): break
                for j in range(len(warehouses)):
                    if (rospy.is_shutdown()): break
                    if i==j:
                        times[i][j] = 0
                        continue
                    print("going from", warehouses[i], "to", warehouses[j])
                    self.go_to(* self.warehouse_info[warehouses[i]]["location"])
                    while not self.truck_in_warehouse(warehouses[i]):
                        time.sleep(0.2)
                    time_start = rospy.get_time()
                    self.go_to(* self.warehouse_info[warehouses[j]]["location"])
                    while not self.truck_in_warehouse(warehouses[j]):
                        time.sleep(0.2)
                    dt = rospy.get_time() - time_start
                    print('result', dt)
                    times[i][j] = dt
                
            
            
            #finished all timing
            print("TIMES", times)

            break

            # self.go_to(* self.warehouse_info[warehouses[idx]]["location"])
            # print("published a message for ",warehouses[idx])
            # #near_boss = True
            # i = 0




            #while (not rospy.is_shutdown()) and i < 15:
                #time.sleep(1)
                #print("my pose", self.truck_x, self.truck_y, "boss pose", self.boss_x, self.boss_y)
                # if(self.near_boss()):
                #     print("near boss")
                #     near_boss = True
                #     self.go_to(self.truck_x, self.truck_y)
                # else:
                #     print("far enough")
                #     if near_boss:
                #         self.go_to(* self.warehouse_info[warehouses[idx]]["location"])
                #         near_boss = False
                #i += 1
            # idx+=1
            # if idx==5: idx=0

    def truck_in_warehouse(self, warehouse_name):
        w_x = self.warehouse_info[warehouse_name]['location'][0]
        w_y = self.warehouse_info[warehouse_name]['location'][1]
        return abs(w_x-self.truck_x)<0.5 and abs(w_y-self.truck_y)<0.25
    
    def near_boss(self):
        return abs(self.boss_x-self.truck_x)<1.5 and abs(self.boss_y-self.truck_y)<1.5
    
    def go_to(self, x_goal, y_goal):
        # todo: get x start and y start from car current position
        #subscribe to slam nodes SLAM/Pose

        # Call the routing service
        plan_request = PlanRequest([self.truck_x, self.truck_y], [x_goal, y_goal])
        plan_response = self.plan_client(plan_request)

        #convert PlanMsg to PathMsg
        #untested
        path_msg = plan_response.path

        path_msg.header.stamp = rospy.get_rostime()
        path_msg.header.frame_id = 'map'

        #Publish PathMsg to a path_topic that traj_planner is listening to 
        self.path_pub.publish(path_msg)











