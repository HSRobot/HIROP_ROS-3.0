#! /usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import random
from hirop_msgs.srv import *
import moveit_commander 
from geometry_msgs.msg import PoseStamped


def save(client, uri, name):
    srv = saveDataEndRequest()
    srv.uri = uri
    srv.name = name
    client.call(srv)


def addData(client, data):
    client.call(data)


def loadData(client, uri, name):
    data = client.call(uri, name)
    return data


def random_go():
    arm.set_random_target()
    arm.go()


def set_pose_go(pose):
    arm.set_pose_target(pose)
    arm.go()


if __name__=="__main__":
    rospy.init_node("test_dm_bridge")
    add_joint_client = rospy.ServiceProxy("add_joint_data", saveJointData)
    load_joint_client = rospy.ServiceProxy("load_joint_data", loadJointsData)

    add_pose_client = rospy.ServiceProxy("add_pose_data", savePoseData)
    load_pose_client = rospy.ServiceProxy("load_pose_data", loadPoseData)

    save_data_end = rospy.ServiceProxy("save_data_end", saveDataEnd)
    set_robot_type = rospy.ServiceProxy("set_robot_type", setRobotType)
    # 设置机器人的信息
    setType = setRobotTypeRequest()
    setType.robot_name = "co605"
    setType.DOF = 6
    set_robot_type.call(setType)
    # ########### 
    testTXT = 0
    arm = moveit_commander.MoveGroupCommander("arm1")
    while not rospy.is_shutdown():
        print("0: 随机改变初始位置")
        print("1: 测试单个关节点位")
        print("2: 测试单个笛卡尔点位")
        print("3: 测试多个关节点位")
        print("4: 测试多个笛卡尔点位")
        testTXT = raw_input("输入测试序号")
        test = eval(testTXT)
        if test == 0:
            print("随机改变初始位置")
            random_go()
        elif test == 1:
            print("测试单个关节点位")
            jointData = arm.get_current_joint_values()
            random_go()
            addData(add_joint_client, jointData)
            save(save_data_end, "joint", "j")
            data = loadData(load_joint_client, "joint", "j")
            arm.set_joint_value_target(data.joints[0].joint)
            arm.go()
        elif test == 2:
            print("测试单个笛卡尔点位")
            PoseData = arm.get_current_pose()
            random_go()
            addData(add_pose_client, PoseData)
            save(save_data_end, "pose", "p")
            data = loadData(load_pose_client, "pose", "p")
            arm.set_pose_target(data.poses[0])
            arm.go()
        elif test == 3:
            print("测试多个关节点位")
            pointCnt = raw_input("输入记录点数")
            pointCnt = eval(pointCnt)
            jointData = arm.get_current_joint_values()
            for i in range(pointCnt):
                addData(add_joint_client, jointData)
                jointData[i%setType.DOF] += 0.01
                arm.set_joint_value_target(jointData)
                arm.go()
            save(save_data_end, "joint", "multiJ")
            data = loadData(load_joint_client, "joint", "multiJ")
            for i in range(pointCnt):
                print(i)
                arm.set_joint_value_target(data.joints[i].joint)
                arm.go()
        elif test == 4:
            print("测试多个笛卡尔点位")
            pointCnt = raw_input("输入记录点数")
            pointCnt = eval(pointCnt)
            poseData = arm.get_current_pose()
            for i in range(pointCnt):
                addData(add_pose_client, poseData)
                poseData.pose.position.x += 0.02
                arm.set_pose_target(poseData)
                arm.go()
            save(save_data_end, "pose", "multiP")
            data = loadData(load_pose_client, "pose", "multiP")
            for i in range(pointCnt):
                print(i)
                arm.set_pose_target(data.poses[i])
                arm.go()
        

