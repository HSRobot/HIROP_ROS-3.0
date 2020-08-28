#pragma once
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include "moveit_msgs/Constraints.h"

struct WorkSpace
{
    double minX;
    double minY;
    double minZ;
    double maxX;
    double maxY;
    double maxZ;
};

struct PlanningConfig
{
    double velocityScalingFactor;
    double accelerationScalingFactor;
    double planningTime;
    std::string plannerID;
    double positionTolerance;
    double orientationTolorance;
    WorkSpace workSpace;
    bool allowReplanning;
    int planningNum;
};



class LoadYaml
{
private:
    int loadJointConstraints(YAML::Node node, std::vector<moveit_msgs::JointConstraint>&);
    int loadPositionConstraints(YAML::Node node, std::vector<moveit_msgs::PositionConstraint>&);
    int loadOrientationConstraints(YAML::Node node, std::vector<moveit_msgs::OrientationConstraint>&);
    // int loadVisibilityConstraints(YAML::Node node, std::vector<moveit_msgs::VisibilityConstraint>&);
public:
    int loadPathConstraints(std::string& path, moveit_msgs::Constraints& con);
    int loadParam(std::string& path, PlanningConfig& config);
    LoadYaml();
    ~LoadYaml();
};


