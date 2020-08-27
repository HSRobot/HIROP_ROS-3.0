#! /usr/bin/env python
import rospy
import moveit_commander
from hirop_msgs.srv import *


if __name__=="__main__":
    rospy.init_node("test_planner")
    arm = moveit_commander.MoveGroupCommander("arm")
    rospy.wait_for_service("/trajectory_planner/getPlannTrajectory")
    client = rospy.ServiceProxy("/trajectory_planner/getPlannTrajectory", getTrajectory)
    while not rospy.is_shutdown():
        i = raw_input("---")
        rep = client()
        # print(rep)
        print(len(rep.tarjectory.joint_trajectory.points))
        print(arm.execute(rep))
        

