#include "planner_bridge/planner.h"

Planner::Planner(std::string& name)
{   
    move_group = new moveit::planning_interface::MoveGroupInterface(name);
    processingYaml = new LoadYaml();
}

Planner::~Planner()
{
    delete move_group;
    delete processingYaml;
}

int Planner::addJointPose(std::vector<std::vector<double> >& joints)
{
    int flag = 0;
    std::vector<moveit_msgs::RobotTrajectory>().swap(midTrajectory);
    move_group->setStartStateToCurrentState();
    for(int i=0; i<joints.size(); i++)
    {
        moveit_msgs::RobotTrajectory tra;
        for(int j=0; j<joints[i].size(); j++)
        {
            joints[i][j] = (joints[i][j] / 180) * M_PI;
        }
        flag = jointPlanner(joints[i], tra);
        if(flag != 0)
        {
            break;
        }
        setNextState(tra);
        midTrajectory.push_back(tra);
    }
    if(flag == 0)
    {
        moveit_msgs::RobotTrajectory midTra;
        getStitchingTrajectory(midTra);
        AdjustTrajectory(midTra);
    }
    return flag;
}


int Planner::addCartesianPose(std::vector<geometry_msgs::PoseStamped>& poses, std::vector<int>& types)
{
    int flag=0;
    std::vector<moveit_msgs::RobotTrajectory>().swap(midTrajectory);
    move_group->setStartStateToCurrentState();
    for(int i=0; i<poses.size(); i++)
    {
        int type;
        if(types.size() == 1)
        {
            type = types[0];
        }
        else
        {
            type = types[i];
        }
        moveit_msgs::RobotTrajectory tra;
        if(type == 0)
            flag = cartesianPlanner(poses[i], tra);
        else
            flag = cartesianLinePlanner(poses[i], tra);
        if(flag != 0)
        {
            break;
        }
        setNextState(tra);
        midTrajectory.push_back(tra);
    }
    if(flag == 0)
    {
        moveit_msgs::RobotTrajectory midTra;
        getStitchingTrajectory(midTra);
        AdjustTrajectory(midTra);
    }
    return flag;
}

int Planner::setPathConstraints(std::string file)
{
    std::string package;
    package = ros::package::getPath("planner_bridge");
    std::string path;
    path = package + "/config/" + file + ".yaml";
    moveit_msgs::Constraints constraints;
    processingYaml->loadPathConstraints(path, constraints);
    move_group->setPathConstraints(constraints);
    ROS_INFO_STREAM(move_group->getPathConstraints());
    return 0;
}

int Planner::clearPathConstraints()
{
    move_group->clearPathConstraints();
    return 0;
}

int Planner::updateParam(std::string file)
{
    std::string package;
    package = ros::package::getPath("planner_bridge");
    std::string path;
    path = package + "/config/" + file + ".yaml";
    processingYaml->loadParam(path, plannerConfig);
    move_group->setMaxVelocityScalingFactor(plannerConfig.velocityScalingFactor);
    move_group->setMaxAccelerationScalingFactor(plannerConfig.accelerationScalingFactor);
    move_group->setPlanningTime(plannerConfig.planningTime);
    if(plannerConfig.plannerID != "NULL")
        move_group->setPlannerId(plannerConfig.plannerID);
    move_group->setGoalPositionTolerance(plannerConfig.positionTolerance);
    move_group->setGoalOrientationTolerance(plannerConfig.orientationTolorance);

    double size[6];
    size[0] = plannerConfig.workSpace.minX;
    size[1] = plannerConfig.workSpace.minY;
    size[2] = plannerConfig.workSpace.minZ;
    size[3] = plannerConfig.workSpace.maxX;
    size[4] = plannerConfig.workSpace.maxY;
    size[5] = plannerConfig.workSpace.maxZ;
    if(size[0] == 0 || size[1] == 0 || size[2] == 0 || 
            size[3] == 0 || size[4] == 0 || size[5] == 0)
    {
        move_group->setWorkspace(size[0], size[1], size[2], size[3], size[4], size[5]);
    }
    move_group->allowReplanning(plannerConfig.allowReplanning);
    return 0;
}

int Planner::getTrajectory(moveit_msgs::RobotTrajectory& tra)
{
    // moveit::planning_interface::MoveGroupInterface::Plan  plan;
    // plan.trajectory_ = targetTrajectory[0];
    // move_group->setStartStateToCurrentState();
    // move_group->execute(plan);
    // ROS_INFO_STREAM(targetTrajectory[0].joint_trajectory.points.size());
    // ROS_INFO_STREAM(targetTrajectory[0].joint_trajectory);
    tra = targetTrajectory[0];
    return 0;
}

int Planner::jointPlanner(std::vector<double>& joint, moveit_msgs::RobotTrajectory& tra)
{
    move_group->setJointValueTarget(joint);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    int cnt = 0;
    bool flag;
    while (ros::ok() && cnt < plannerConfig.planningNum)
    {
        flag = move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        if(flag == true)
        {
            tra = my_plan.trajectory_;
            break;
        }
        cnt ++;
    }
    if(!flag)
        return -1;
    return 0;
}

int Planner::cartesianLinePlanner(geometry_msgs::PoseStamped& pose, moveit_msgs::RobotTrajectory& tra)
{
    std::vector<geometry_msgs::Pose> wayPoint;
    moveit_msgs::RobotTrajectory planTra;
    wayPoint.push_back(pose.pose);
    double eef_step = 0.01; 
    double jump_threshold = 0;
    int cnt = 0;
    double flag;
    while (ros::ok() && cnt < plannerConfig.planningNum)
    {
        flag = move_group->computeCartesianPath(wayPoint, eef_step, jump_threshold, planTra);
        if(flag >= 0.8)
        {
            tra = planTra;
            break;
        }
        cnt ++;
    }
    if(flag < 0.8)
        return -1;
    return 0;
    
}

int Planner::cartesianPlanner(geometry_msgs::PoseStamped& pose, moveit_msgs::RobotTrajectory& tra)
{
    move_group->setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    int cnt = 0;
    bool flag;
    while (ros::ok() && cnt < plannerConfig.planningNum)
    {
        flag = move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        if(flag == true)
        {
            tra = my_plan.trajectory_;
            break;
        }
        cnt ++;
    }
    if(!flag)
        return -1;
    return 0;
}

int Planner::getStitchingTrajectory(moveit_msgs::RobotTrajectory& tra)
{
    int count = midTrajectory.size();
    for(int i=0; i<count; i++)
    {

        int pointCount = 0;
        if(i != (count-1))
        {
            pointCount = midTrajectory[i].joint_trajectory.points.size() - 1;
        }
        else
        {
            pointCount = midTrajectory[i].joint_trajectory.points.size();
        }
        tra.joint_trajectory.joint_names = midTrajectory[0].joint_trajectory.joint_names;
        for(int j=0; j<pointCount; j++)
        {
            tra.joint_trajectory.points.push_back(midTrajectory[i].joint_trajectory.points[j]);
        }
    }
    return 0;
}

int Planner::AdjustTrajectory(moveit_msgs::RobotTrajectory& tra)
{
    robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), move_group->getName());
    rt.setRobotTrajectoryMsg(*(move_group->getCurrentState()), tra);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, plannerConfig.velocityScalingFactor, plannerConfig.accelerationScalingFactor);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // std::vector<moveit_msgs::RobotTrajectory> tra2;
    // tra2.resize(1);
    std::vector<moveit_msgs::RobotTrajectory>().swap(targetTrajectory);
    targetTrajectory.resize(1);
    rt.getRobotTrajectoryMsg(targetTrajectory[0]);
    return 0;
}

int Planner::setNextState(moveit_msgs::RobotTrajectory& tra)
{
    moveit_msgs::RobotState state;
    state.joint_state.header.frame_id = tra.joint_trajectory.header.frame_id;
    state.joint_state.name = tra.joint_trajectory.joint_names;
    int size = tra.joint_trajectory.points.size();
    state.joint_state.position = tra.joint_trajectory.points[size-1].positions;
    state.joint_state.velocity = tra.joint_trajectory.points[size-1].velocities;
    state.joint_state.effort = tra.joint_trajectory.points[size-1].effort;
    move_group->setStartState(state);
    ros::Duration(0.25).sleep();
    return 0;
}

int Planner::str2Num(std::string str)
{
    int index = -1;
    boost::regex reg("joint([0-9]+)");
    boost::smatch results;
    if(boost::regex_match(str, results, reg))
    {
        auto i = results[1];
        boost::ssub_match result = i;
        std::string strNum = result.str();
        std::stringstream ss(strNum);
        ss >> index;
    }
    return index;
}

int Planner::singleAxis(std::vector<std::string> axisIndex, std::vector<double> incrementAngle)
{
    int flag = 0;
    std::vector<moveit_msgs::RobotTrajectory>().swap(midTrajectory);
    move_group->setStartStateToCurrentState();
    std::vector<double> joints = move_group->getCurrentJointValues();
    for(int i=0; i<axisIndex.size(); i++)
    {
        int index = str2Num(axisIndex[i]);
        if(index == -1)
        {
            flag = -1;
            break;
        }
        index --;
        double radin = (incrementAngle[i] / 180) * M_PI;
        joints[index] += radin;
        moveit_msgs::RobotTrajectory tra;
        flag = jointPlanner(joints, tra);
        if(flag != 0)
        {
            break;
        }
        setNextState(tra);
        midTrajectory.push_back(tra);
        int size = tra.joint_trajectory.points.size();
        joints = tra.joint_trajectory.points[size - 1].positions;
    }
    if(flag == 0)
    {
        moveit_msgs::RobotTrajectory midTra;
        getStitchingTrajectory(midTra);
        AdjustTrajectory(midTra);
    }
    return flag;
}

int Planner::IK(geometry_msgs::PoseStamped& pose, std::vector<double>& joints, std::string tip)
{
    int flag = -1;
    moveit::core::RobotStatePtr robot_state = move_group->getCurrentState();
    const robot_model::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(move_group->getName());
    std::size_t attempts = 10;
    double timeout = 0.5;
    int cnt = 0;
    while (ros::ok() && cnt < 5)
    {
        // if(robot_state->setFromIK(joint_model_group, pose.pose, attempts, timeout))
        if(robot_state->setFromIK(joint_model_group, pose.pose, tip, attempts, timeout))
        {
            ROS_INFO_STREAM("IK succeed");
            flag = 0;
            robot_state->copyJointGroupPositions(joint_model_group, joints);
            break;
        }
        cnt ++;
        ROS_INFO_STREAM("IK cnt: " << cnt);
    }
    return flag;
}
    