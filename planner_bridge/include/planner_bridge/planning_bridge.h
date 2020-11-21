#pragma once
#include <std_msgs/Empty.h>
#include <math.h>
#include "hirop_msgs/addPose.h"
#include "hirop_msgs/addJointPose.h"
#include "hirop_msgs/updateParam.h"
#include "hirop_msgs/getTrajectory.h"
#include "hirop_msgs/incrementAngle.h"
#include "hirop_msgs/setFromIK.h"
#include "planner.h"

class PlanningBridge
{
private:
    bool singleAxisCB(hirop_msgs::incrementAngle::Request& req, hirop_msgs::incrementAngle::Response& rep);
    bool addCartesianPoseCB(hirop_msgs::addPose::Request& req, hirop_msgs::addPose::Response& rep);
    bool addJointPoseCB(hirop_msgs::addJointPose::Request& req, hirop_msgs::addJointPose::Response& rep);
    bool updateParamCB(hirop_msgs::updateParam::Request& req, hirop_msgs::updateParam::Response& rep);
    bool getTrajectoryCB(hirop_msgs::getTrajectory::Request& req, hirop_msgs::getTrajectory::Response& rep);
    bool setPathConstraintsCB(hirop_msgs::updateParam::Request& req, hirop_msgs::updateParam::Response& rep);
    bool IKCB(hirop_msgs::setFromIK::Request& req, hirop_msgs::setFromIK::Response& rep);
    void clearPathConstraintsCB(const std_msgs::EmptyConstPtr& msg);

    std::vector<double> rad2angle(std::vector<double> rad);

    // 单轴增量
    ros::ServiceServer singleAxisServer;
    // 规划笛卡尔坐标
    ros::ServiceServer addCartesianPose;
    // 规划关节坐标
    ros::ServiceServer addJointPose;
    // 更新规划参数
    ros::ServiceServer updateParam;
    // 获取轨迹
    ros::ServiceServer getTrajectory;
    // 设置路径约束
    ros::ServiceServer setPathConstraints;
    ros::ServiceServer setFromIKServer;
    // 清楚路径约束
    ros::Subscriber clearPathConstraints;

    ros::NodeHandle& nh;

    Planner* planner;

    std::string tip;


public:
    PlanningBridge(ros::NodeHandle& n);
    ~PlanningBridge();
};


