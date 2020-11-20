#! /usr/bin/env python

# import roslib
import rospy
import actionlib
from std_msgs.msg import Bool
from hirop_msgs.srv import *
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import JointState
import time
import thread

finger_client = None
open_server = 'openGripper'
close_server = 'closeGripper'
finger_server = "moveSeq"

current_gripper_state = False
def callback(req):
    try:
        if req.data:
            finger_client(4)
        elif not req.data:
            finger_client(2)
        return SetBoolResponse(True, "true")
    except:
        pass

if __name__ == '__main__':
    rospy.init_node('gripper_serial')
    rospy.wait_for_service(finger_server)
    move_group = rospy.get_param("~move_group_name", 'arm1')
    gripper_sub = move_group + "/gripper_state"
    try:
        s = rospy.Service(gripper_sub, SetBool, callback)
        print("server is ok")
        finger_client = rospy.ServiceProxy(finger_server, moveSeqIndex)
        rospy.spin()
    except:
        pass
    