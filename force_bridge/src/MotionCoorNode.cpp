#include <iostream>
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <MotionCoorServer.h>
using namespace std;

////服务回调函数 //设置阻抗随动方向

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motionCoorControl");
    ros::NodeHandle node;
    ros::AsyncSpinner as(1);
    as.start();
    MotionCoorServer ser(node);
    ser.start();
    //初始化服务与话题
//    ros::ServiceServer motionCoorSet_server = node.advertiseService("motionCoorControl_set", motionCoorSetCB);
//    ros::ServiceServer motionCoorShutDown_server = node.advertiseService("motionCoorControl_shutdown", motionCoorShutDownCB);
//    ros::Subscriber pub_rob  =node.subscribe("/plan/joint_path_command",1);

    return 0;
}
