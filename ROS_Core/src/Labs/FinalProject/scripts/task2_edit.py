#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import yaml
from task2_world.util import get_ros_param
from final_project.srv import Schedule, ScheduleRequest, ScheduleResponse
from final_project.srv import Task, TaskRequest, TaskResponse
from final_project.srv import Reward, RewardRequest, RewardResponse

# Pull in all needed info/services as descirbed in readme file

##### Start swifthaul via call to service
rospy.wait_for_service('/SwiftHaul/Start')
start_client = rospy.ServiceProxy('/SwiftHaul/Start', Empty)
# call the service
start_client(EmptyRequest())

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

#We consider the truck is in the warehouse when it is 
# less than 0.5 meters away in longitudinal direction and 
# 0.25 meters away in lateral direction.

def truck_in_warehouse(x, y, warehouse_name, warehouse_info):
    w_x = warehouse_info[warehouse_name]['location'][0]
    w_y = warehouse_info[warehouse_name]['location'][1]
    return abs(w_x-x)<0.5 and abs(w_y-y)<0.25




##### Get Boss Schedule
rospy.wait_for_service('/SwiftHaul/BossSchedule')
boss_sched_client = rospy.ServiceProxy('/SwiftHaul/BossSchedule', Schedule)
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
sidetask_client = rospy.ServiceProxy('/SwiftHaul/SideTask', Task)

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
bosstask_client = rospy.ServiceProxy('/SwiftHaul/BossTask', Task)

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
reward_client = rospy.ServiceProxy('/SwiftHaul/GetReward', Reward)

# how to call the service
# respond = reward_client(RewardRequest())
# call AFTER completion to get reward
# The respond of this Reward service has two fields:
# respond.done # bool, the status of the task. If the task is done, it will be True
# respond.total_reward # float, the total reward of so far


print("Set up task 2 node successfully")





