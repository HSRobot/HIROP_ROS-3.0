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

open_client = None
close_client = None
finger_client = None
open_server = 'openGripper'
close_server = 'closeGripper'
finger_server = "moveSeq"

current_gripper_state = False
def callback(req):
    try:
        if req.data:
            rospy.loginfo("---------------->open gripper")
            open_client()
        elif not req.data:
            rospy.loginfo("---------------->close gripper")
            close_client()
            rospy.Duration(1).sleep()
        return SetBoolResponse(True, "true")
    except:
        pass

if __name__ == '__main__':
    rospy.init_node('gripper_serial')
    rospy.wait_for_service(open_server)
    move_group = rospy.get_param("~move_group_name", 'arm')
    gripper_ = move_group + "/gripper_state"
    try:
        s = rospy.Service(gripper_, SetBool, callback)
        print("server is ok")
        open_client = rospy.ServiceProxy(open_server, openGripper)
        close_client = rospy.ServiceProxy(close_server, closeGripper)
        rospy.spin()
    except:
        pass
    
