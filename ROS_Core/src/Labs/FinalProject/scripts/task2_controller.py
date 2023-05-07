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

        self.boss_recent_positions = []
        self.boss_pos_track_counter = 0
        self.track_pos_every_x_messages = 5
        self.boss_safe_dist = 0.3
        self.safe_boss_target_x = 0
        self.safe_boss_target_y = 0



        ##### Get Warehouse Info
        warehouse_yaml = get_ros_param("/task2_world_node/warehouse_yaml", None)
        with open(warehouse_yaml, "r") as stream:
            warehouse_info = yaml.safe_load(stream)
        #print("WAREHOUSE INFO",warehouse_info)
        #output:
        # {'warehouse_A': {'id': 0, 'location': [3, 0.15], 'dxdy': [0.5, 0.25], 'probability': [0.3, 0.05, 0.15, 0.3, 0.2]}, 
        # 'warehouse_B': {'id': 1, 'location': [5.9, 3.5], 'dxdy': [0.25, 0.5], 'probability': [0.1, 0.4, 0.2, 0.2, 0.1]}, 
        # 'warehouse_C': {'id': 2, 'location': [0.15, 3.5], 'dxdy': [0.25, 0.5], 'probability': [0.5, 0.3, 0.1, 0.05, 0.05]}, 
        # 'warehouse_D': {'id': 3, 'location': [3, 2.2], 'dxdy': [0.5, 0.5], 'probability': [0.2, 0.2, 0.3, 0.1, 0.2]}, 
        # 'warehouse_E': {'id': 4, 'location': [3, 1.1], 'dxdy': [0.5, 0.25], 'probability': [0.2, 0.1, 0.3, 0.25, 0.15]}}
        self.warehouses = []
        for k in warehouse_info.keys():
            self.warehouses.append(warehouse_info[k])
            self.warehouses[-1]["name"] = k
        #now, have list of dictionaries accessible by index, but can still get name

        

        #We consider the truck is in the warehouse when it is 
        # less than 0.5 meters away in longitudinal direction and 
        # 0.25 meters away in lateral direction.

        #warehouse_distances[i][j] is the approx rospy time it takes to get from warehouse i to warehouse j
        self.warehouse_distances = [
            [0.0, 8.6, 18.7, 16.3, 21.9],
            [16.7, 0.0, 12.6, 10.4, 16.0],
            [6.8, 14.0, 0.0, 21.9, 26.9],
            [15.8, 22.9, 11.6, 0.0, 6.7],
            [20.7, 27.7, 16.7, 3.0, 0.0]
            ]


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

        ### basic navigation/world ###
        print("try to wait for ilqr warmup")
        rospy.wait_for_service('/planning/start_planning')

        rospy.wait_for_service('/routing/plan')
        self.plan_client = rospy.ServiceProxy('/routing/plan', Plan)


        self.odom_topic = get_ros_param('~odom_topic', '/Simulation/Pose')
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)
        self.boss_pose_sub = rospy.Subscriber("/Boss/Pose", Odometry, self.boss_pose_callback, queue_size=10)
       
        # set up publisher for trajectory received from routing
        self.path_topic = get_ros_param('~path_topic', '/Routing/Path')
        self.path_pub = rospy.Publisher(self.path_topic, PathMsg, queue_size=1)

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

        #only record, for example, every 5th message
        if self.boss_pos_track_counter == 0:
            self.boss_recent_positions.insert(0, [self.boss_x, self.boss_y])
            # check if there are more messages on the end than we need
            #iterate backwards
            counter = 1
            while (counter < len(self.boss_recent_positions)): #say array is length 5, stop at arr[-4] which is the second element
                if self.dist_to_boss(self.boss_recent_positions[-1 * counter]) < self.boss_safe_dist:
                    break
                counter += 1
            #now, counter is set to the first 'dangerous' index from the end 
            # say counter=4 that means the last 3 items are "safe"
            # we only need 1 safe item
            items_to_remove = counter - 2
            while(items_to_remove > 0):
                self.boss_recent_positions.pop()
                items_to_remove -= 1
            
            #now we have at least 1 safe item at the end of the list
            if counter != 1:
                self.safe_boss_target_x = self.boss_recent_positions[-1][0]
                self.safe_boss_target_y = self.boss_recent_positions[-1][1]

        self.boss_pos_track_counter += 1
        if self.boss_pos_track_counter == self.track_pos_every_x_messages:
            self.boss_pos_track_counter = 0


    def loop(self):
        #warehouses = self.warehouses
        #idx = 0
        #self.go_to(* self.warehouse_info[warehouses[idx]]["location"])
        doing_boss_task = False
        task = -1
        planned_reward = 0

        while rospy.is_shutdown() is False and (self.safe_boss_target_x == self.safe_boss_target_y): 
            time.sleep(1)
            print("waiting for safe boss target...")
        while rospy.is_shutdown() is False:  #check shutdown condition in ALL LOOPS
            #self.go_to_warehouse(idx)
            doing_boss_task = False
            task = -1
            planned_reward = 0
            while not doing_boss_task and not rospy.is_shutdown():
                task, planned_reward = self.get_boss_task()
                if task != -1:
                    doing_boss_task = True
                time.sleep(1)
                print("waiting for boss task...")
            
            if doing_boss_task:
                timeout = 45
                while timeout > 0 and not self.truck_in_warehouse(task):
                    self.go_to_warehouse(task)
                    timeout -= 5
                    time.sleep(5)
                if timeout == 0:
                    print("failed this task")
                    continue
                done, total = self.collect_reward(task)
                if not done:
                    print("It didn't think we were actually done with the task...")
                else:
                    print("Finished task. Total reward so far is ", total)
                
                

            # print("following boss to ",self.safe_boss_target_x, self.safe_boss_target_y)
            # self.go_to(self.safe_boss_target_x, self.safe_boss_target_y)
            # time.sleep(5)


    def truck_in_warehouse(self, w_idx):
        if w_idx == -1: return False
        w_loc = self.warehouses[w_idx]["location"]
        in_warehouse = abs(w_loc[0]-self.truck_x)<0.5 and abs(w_loc[1]-self.truck_y)<0.25
        print("truck is in",self.warehouses[w_idx]["name"])
        return in_warehouse
    
    def dist_between(self, w1, w2):
        return self.warehouse_distances[w1][w2]

    def go_to_warehouse(self, w_idx):
        print("going to ",self.warehouses[w_idx]["name"])
        self.go_to(* self.warehouses[w_idx]["location"])

    def go_to(self, x_goal, y_goal):
        #subscribe to slam nodes SLAM/Pose if you want physical truck

        # Call the routing service
        plan_request = PlanRequest([self.truck_x, self.truck_y], [x_goal, y_goal])
        plan_response = self.plan_client(plan_request)

        #convert PlanMsg to PathMsg
        path_msg = plan_response.path

        path_msg.header.stamp = rospy.get_rostime()
        path_msg.header.frame_id = 'map'

        #Publish PathMsg to a path_topic that traj_planner is listening to 
        self.path_pub.publish(path_msg)

    def get_boss_task(self):
        respond = self.bosstask_client(TaskRequest())
        return respond.task, respond.reward

    def get_side_task(self):
        pass

    def get_boss_schedule(self):
        pass

    def collect_reward(self,task):
        respond = self.reward_client(RewardRequest(task))
        return respond.done, respond.total_reward

    def dist_to_boss(self, p1):
        dist = np.linalg.norm([p1[0] - self.boss_x, p1[1] - self.boss_y])
        return dist











