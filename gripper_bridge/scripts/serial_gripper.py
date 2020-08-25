#! /usr/bin/env python

import roslib
import rospy
import actionlib
from std_msgs.msg import Bool
from hirop_msgs.srv import *
from sensor_msgs.msg import JointState
import time
import thread


close_client = None
open_client = None
joint_names = ['left_finger_1_joint', 'left_finger_2_joint', 'right_finger_1_joint', 'right_finger_2_joint']
last_joint_state = [0, 0, 0, 0]
open_server = 'openGripper'
close_server = 'closeGripper'
# gripper_sub = "/joint_state"
# action_server = "gripper_controller/follow_joint_trajectory"

current_gripper_state = False
def callback(msg):
    rospy.loginfo("gripper controller")
    if msg.data:
        rospy.loginfo("open gripper")
        open_client()
    elif not msg.data:
        rospy.loginfo("close gripper")
        close_client()

if __name__ == '__main__':
    rospy.init_node('gripper_serial')

    rospy.wait_for_service(open_server)
    rospy.wait_for_service(close_server)
    try:
        move_group = rospy.get_param('/serial_gripper/move_group')
        print(move_group)
        gripper_sub = move_group + "/gripper_state"
        print gripper_sub
        rospy.Subscriber(gripper_sub, Bool, callback)
        print("server is ok")

        open_client = rospy.ServiceProxy(open_server, openGripper)
        close_client = rospy.ServiceProxy(close_server, closeGripper)
        rospy.spin()
    except:
        rospy.spin()


