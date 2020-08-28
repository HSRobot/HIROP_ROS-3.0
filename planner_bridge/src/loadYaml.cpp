#include "planner_bridge/loadYaml.h"

LoadYaml::LoadYaml(/* args */)
{
}

LoadYaml::~LoadYaml()
{
}

int LoadYaml::loadJointConstraints(YAML::Node node, std::vector<moveit_msgs::JointConstraint>& joint)
{
    joint.resize(node.size());
    for(int i=0; i<node.size(); i++)
    {
        joint[i].joint_name = node[i]["joint_name"].as<std::string>();
        joint[i].position = node[i]["position"].as<double>();
        joint[i].tolerance_above = node[i]["tolerance_above"].as<double>();
        joint[i].tolerance_below = node[i]["tolerance_below"].as<double>();
        joint[i].weight = node[i]["weight"].as<double>();
    }
    return 0;
}

int LoadYaml::loadPositionConstraints(YAML::Node node, std::vector<moveit_msgs::PositionConstraint>& posetion)
{
    posetion.resize(node.size());
    for(int i=0; i<node.size(); i++)
    {
        posetion[i].header.frame_id = node[i]["header"]["frame_id"].as<std::string>();
        posetion[i].link_name = node[i]["link_name"].as<std::string>();
        posetion[i].target_point_offset.x = node[i]["target_point_offset"]["x"].as<double>();
        posetion[i].target_point_offset.y = node[i]["target_point_offset"]["y"].as<double>();
        posetion[i].target_point_offset.z = node[i]["target_point_offset"]["z"].as<double>();
        posetion[i].constraint_region.primitives.resize(1);
        posetion[i].constraint_region.primitives[0].type = node[i]["constraint_region"]["primitives"]["type"].as<int>();
        int size = node[i]["constraint_region"]["primitives"]["dimensions"].size();
        posetion[i].constraint_region.primitives[0].dimensions.resize(size);
        for(int j=0; j<size; j++)
        {
            posetion[i].constraint_region.primitives[0].dimensions[j] = node[i]["constraint_region"]["primitives"]["dimensions"][j].as<double>();
        }
        posetion[i].constraint_region.primitive_poses.resize(1);
        posetion[i].constraint_region.primitive_poses[0].position.x = node[i]["constraint_region"]["primitive_poses"]["position"]["x"].as<double>();
        posetion[i].constraint_region.primitive_poses[0].position.y = node[i]["constraint_region"]["primitive_poses"]["position"]["y"].as<double>();
        posetion[i].constraint_region.primitive_poses[0].position.z = node[i]["constraint_region"]["primitive_poses"]["position"]["z"].as<double>();
        posetion[i].constraint_region.primitive_poses[0].orientation.x = node[i]["constraint_region"]["primitive_poses"]["orientation"]["x"].as<double>();
        posetion[i].constraint_region.primitive_poses[0].orientation.y = node[i]["constraint_region"]["primitive_poses"]["orientation"]["y"].as<double>();
        posetion[i].constraint_region.primitive_poses[0].orientation.z = node[i]["constraint_region"]["primitive_poses"]["orientation"]["z"].as<double>();
        posetion[i].constraint_region.primitive_poses[0].orientation.w = node[i]["constraint_region"]["primitive_poses"]["orientation"]["w"].as<double>();
        posetion[i].weight = node[i]["weight"].as<double>();
    }
    return 0;
}

int LoadYaml::loadOrientationConstraints(YAML::Node node, std::vector<moveit_msgs::OrientationConstraint>& orientation)
{
    orientation.resize(node.size());
    for(int i=0; i<node.size(); i++)
    {
        orientation[i].header.frame_id = node[i]["header"]["frame_id"].as<std::string>();
        orientation[i].orientation.x = node[i]["orientation"]["x"].as<double>();
        orientation[i].orientation.y = node[i]["orientation"]["y"].as<double>();
        orientation[i].orientation.z = node[i]["orientation"]["z"].as<double>();
        orientation[i].orientation.w = node[i]["orientation"]["w"].as<double>();
        orientation[i].link_name = node[i]["link_name"].as<std::string>();
        orientation[i].absolute_x_axis_tolerance = node[i]["absolute_x_axis_tolerance"].as<double>();;
        orientation[i].absolute_y_axis_tolerance = node[i]["absolute_y_axis_tolerance"].as<double>();;
        orientation[i].absolute_z_axis_tolerance = node[i]["absolute_z_axis_tolerance"].as<double>();;
        orientation[i].weight =  node[i]["weight"].as<double>();
    }
}

// int LoadYaml::loadVisibilityConstraints(YAML::Node& node, std::vector<moveit_msgs::VisibilityConstraint>& visibility)
// {

// }


int LoadYaml::loadPathConstraints(std::string& path, moveit_msgs::Constraints& con)
{
    YAML::Node node;
    node = YAML::LoadFile(path);
    ROS_INFO_STREAM("constraints file: " << path);
    if(node["joint_constraints"]["switch"].as<bool>())
        loadJointConstraints(node["joint_constraints"]["config"], con.joint_constraints);
    if(node["position_constraints"]["switch"].as<bool>())
        loadPositionConstraints(node["position_constraints"]["config"], con.position_constraints);
    if(node["orientation_constraints"]["switch"].as<bool>())
        loadOrientationConstraints(node["orientation_constraints"]["config"], con.orientation_constraints);
    return 0;
}

int LoadYaml::loadParam(std::string& path, PlanningConfig& config)
{
    YAML::Node node;
    ROS_INFO_STREAM("config file: " << path);
    node = YAML::LoadFile(path);
    config.velocityScalingFactor = node["velocityScalingFactor"].as<double>();
    config.accelerationScalingFactor = node["accelerationScalingFactor"].as<double>();
    config.planningTime = node["planningTime"].as<double>();
    config.plannerID = node["plannerID"].as<std::string>();
    config.positionTolerance = node["positionTolerance"].as<double>();
    config.orientationTolorance = node["orientationTolorance"].as<double>();
    config.allowReplanning = node["allowReplanning"].as<bool>();
    config.planningNum = node["planningNum"].as<int>();

    config.workSpace.minX = node["workSpace"]["minX"].as<double>();
    config.workSpace.minY = node["workSpace"]["minY"].as<double>();
    config.workSpace.minZ = node["workSpace"]["minZ"].as<double>();
    config.workSpace.maxX = node["workSpace"]["maxX"].as<double>();
    config.workSpace.maxY = node["workSpace"]["maxY"].as<double>();
    config.workSpace.maxZ = node["workSpace"]["maxZ"].as<double>();
    return 0;
}
