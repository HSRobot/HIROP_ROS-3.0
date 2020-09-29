#! /usr/bin/env python
import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from hirop_msgs.srv import *


if __name__=="__main__":
    rospy.init_node("test_planner")
    arm = moveit_commander.MoveGroupCommander("arm")
    rospy.wait_for_service("/trajectory_planner/getPlannTrajectory")
    motion_client = rospy.ServiceProxy("/motion_bridge/moveToMultiPose", moveToMultiPose)
    client = rospy.ServiceProxy("/trajectory_planner/getPlannTrajectory", getTrajectory)
    while not rospy.is_shutdown():
        i = raw_input("---")
        rep = client()

        # tra = RobotTrajectory()
        # tra.joint_trajectory.header = rep.tarjectory.joint_trajectory.header
        # tra.joint_trajectory.joint_names = rep.tarjectory.joint_trajectory.joint_names

        # print(num = len(rep.tarjectory.joint_trajectory.points))
        # for j in rep.tarjectory.joint_trajectory.points:
        #     tra.joint_trajectory.points.append(j)

        # motion_req = moveToMultiPoseRequest()
        # motion_req.moveGroup_name = "arm"
        # for i in tra.joint_trajectory.points:
        #     motion_req.poseList_joints_angle.append()
        # print(motion_client(motion_req))
        print(len(rep.tarjectory.joint_trajectory.points))
	# print(rep.tarjectory.joint_trajectory)
        print(arm.execute(rep))
        

