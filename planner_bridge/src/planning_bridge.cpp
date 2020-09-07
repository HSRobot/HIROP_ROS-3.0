#include "planner_bridge/planning_bridge.h"

PlanningBridge::PlanningBridge(ros::NodeHandle& n)
:nh{n}
{
    std::string move_group_name;
    std::string configFile;
    nh.param("move_group_name", move_group_name, std::string("arm"));
    nh.param("config", configFile, std::string("config"));
    planner = new Planner(move_group_name);
    planner->updateParam(configFile);
    singleAxisServer = nh.advertiseService("sigleIncrementAngle", &PlanningBridge::singleAxisCB, this);
    addCartesianPose = nh.advertiseService("cartesianPlanner", &PlanningBridge::addCartesianPoseCB, this);
    addJointPose = nh.advertiseService("jointSpacePlanner", &PlanningBridge::addJointPoseCB, this);
    updateParam = nh.advertiseService("updatePlannerParam", &PlanningBridge::updateParamCB, this);
    getTrajectory = nh.advertiseService("getPlannTrajectory", &PlanningBridge::getTrajectoryCB, this);
    setPathConstraints = nh.advertiseService("setPlannerPathConstraints", &PlanningBridge::setPathConstraintsCB, this);
    setFromIKServer = nh.advertiseService("setFromIK", &PlanningBridge::IKCB, this);
    clearPathConstraints = nh.subscribe("clearPlannerPathConstraints", 10, &PlanningBridge::clearPathConstraintsCB, this);
}

PlanningBridge::~PlanningBridge()
{
    delete planner;
}

bool PlanningBridge::singleAxisCB(hirop_msgs::incrementAngle::Request& req, hirop_msgs::incrementAngle::Response& rep)
{
    bool flag;
    rep.result = planner->singleAxis(req.index, req.incrementAngle);
	return true;
}

bool PlanningBridge::addCartesianPoseCB(hirop_msgs::addPose::Request& req, hirop_msgs::addPose::Response& rep)
{
    rep.result = planner->addCartesianPose(req.pose, req.type);
    return true;
}

bool PlanningBridge::addJointPoseCB(hirop_msgs::addJointPose::Request& req, hirop_msgs::addJointPose::Response& rep)
{
    std::vector<std::vector<double> > joints;
    joints.resize(req.joints.size());
    for(int i=0; i<joints.size(); i++)
    {
        joints[i].resize(req.joints[i].joint.size());
        joints[i] = req.joints[i].joint;
    }
    rep.result = planner->addJointPose(joints);
    return true;
}

bool PlanningBridge::updateParamCB(hirop_msgs::updateParam::Request& req, hirop_msgs::updateParam::Response& rep)
{
    int flag;
    flag = planner->updateParam(req.file);
    if(flag == 0)
    {
        rep.result = true;
    }
    else
    {
        rep.result = false;
    }
    return true;
}

bool PlanningBridge::getTrajectoryCB(hirop_msgs::getTrajectory::Request& req, hirop_msgs::getTrajectory::Response& rep)
{
    planner->getTrajectory(rep.tarjectory);
    return true;
}

bool PlanningBridge::setPathConstraintsCB(hirop_msgs::updateParam::Request& req, hirop_msgs::updateParam::Response& rep)
{
    planner->setPathConstraints(req.file);
    rep.result = true;
    return true;
}

bool PlanningBridge::IKCB(hirop_msgs::setFromIK::Request& req, hirop_msgs::setFromIK::Response& rep)
{
    std::vector<double> joints;
    int flag = -1;
    flag = planner->IK(req.Pose, joints);
    if(flag == 0)
    {
        rep.joints = joints;
        return true;
    }
    return false;
}

void PlanningBridge::clearPathConstraintsCB(const std_msgs::EmptyConstPtr& msg)
{
    planner->clearPathConstraints();
}

