#! /usr/bin/env python

import rospy
from hirop_msgs.srv import *
from sensor_msgs.msg import JointState
import moveit_commander


open_server = 'openGripper'
close_server = 'closeGripper'

current_state = False

def callback(msg):
    global current_state
    # print(msg.position[6])
    if msg.position[6] > -0.1 and msg.position[6] < 0.1 and current_state == False:
        current_state = True
        rospy.loginfo("open gripper")
        # open_client()
    elif msg.position[6] > 0.1 and current_state == True:
        current_state = False
        rospy.loginfo("close gripper")
        # close_client()


if __name__=="__main__":
    rospy.init_node('gripper_action_server')
    open_client = rospy.ServiceProxy(open_server, openGripper)
    close_client = rospy.ServiceProxy(close_server, closeGripper)
    sub = rospy.Subscriber('/joint_states', JointState, callback)
    scene = moveit_commander.PlanningSceneInterface()
    # ls = ["object"]
    # while not rospy.is_shutdown():
    #     obj = scene.get_attached_objects(ls)
    #     if len(obj) == 0:
    #         continue
    #     print(obj["object"])
    rospy.spin()