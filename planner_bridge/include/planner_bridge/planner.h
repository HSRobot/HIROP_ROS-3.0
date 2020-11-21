#pragma once
#include <ros/package.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <math.h>
#include "loadYaml.h"
#include <iostream>
#include <sstream>
#include <boost/regex.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>

class Planner
{
private:
    int jointPlanner(std::vector<double>& joint, moveit_msgs::RobotTrajectory& tra);
    int cartesianLinePlanner(geometry_msgs::PoseStamped& poses, moveit_msgs::RobotTrajectory& tra);
    int cartesianPlanner(geometry_msgs::PoseStamped& pose, moveit_msgs::RobotTrajectory& tra);
    int getStitchingTrajectory(moveit_msgs::RobotTrajectory& tra);
    int AdjustTrajectory(moveit_msgs::RobotTrajectory& tra);
    int setNextState(moveit_msgs::RobotTrajectory& tra);
    int str2Num(std::string str);

    PlanningConfig plannerConfig;
    

    LoadYaml* processingYaml;
    
    moveit::planning_interface::MoveGroupInterface* move_group;
    std::vector<moveit_msgs::RobotTrajectory> targetTrajectory;
    std::vector<moveit_msgs::RobotTrajectory> midTrajectory;
    
public:
    Planner(std::string& move_name);
    ~Planner();

    /**
     * @brief 进行关节空间规划
     * @param joints 需要进行规划的系列关节坐标
     * @return 成功返回0
    */
    int addJointPose(std::vector<std::vector<double> >& joints);

    /**
     * @brief 进行笛卡尔空间规划
     * @param poses 需要进行规划的系列笛卡尔空间坐标
     * @param types 需要进行的规划类型, 0为曲线, 1为直线
     * @return 成功返回0
    */
    int addCartesianPose(std::vector<geometry_msgs::PoseStamped>& poses, std::vector<int>& types);

    /**
     * @brief 设置路径规划
     * @param fileName 需要加载的约束文件名称
    */
    int setPathConstraints(std::string fileName);

    /**
     * @brief 清楚路径约束
    */
    int clearPathConstraints();

    /**
     * @brief 刷新规划器参数
     * @param fileName 需要加载的约束文件名称
    */
    int updateParam(std::string fileName);

    /**
     * @brief 获取规划成功的经过连接的轨迹
     * @param tra 输出的轨迹
    */
    int getTrajectory(moveit_msgs::RobotTrajectory& tra);

    int singleAxis(std::vector<std::string> axisIndex, std::vector<double> incrementAngle);

    /**
     * @brief 
    */
    int IK(geometry_msgs::PoseStamped& pose, std::vector<double>& joint, std::string tip);
};


