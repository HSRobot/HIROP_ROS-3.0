#pragma once
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <hirop_msgs/motionCoorStart.h>
#include <hirop_msgs/motionCoorStop.h>
#include <queue>
#include <boost/atomic.hpp>
#include <boost/thread.hpp>
class MotionCoorServer
{
public:
    MotionCoorServer(ros::NodeHandle &nh);
    void start();
private:
    bool motionCoorStartCB(hirop_msgs::motionCoorStartRequest & req, hirop_msgs::motionCoorStartResponse& res );
    bool motionCoorStopCB(hirop_msgs::motionCoorStopRequest & req, hirop_msgs::motionCoorStopResponse& res );
    void pathTrajPointCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
    void threadRun();
private:
    ros::NodeHandle nh;
    ros::ServiceServer motionCoorSet_server;
    ros::ServiceServer motionCoorShutDown_server;
    ros::ServiceClient motionImpedenceStartCli,motionImpedenceStopCli;
    ros::Subscriber pub_rob;
    std::queue<trajectory_msgs::JointTrajectory> jointTrajQueue;
    boost::atomic<bool> stop;
    boost::atomic<bool> isRunning;
    boost::condition_variable condition;
};
