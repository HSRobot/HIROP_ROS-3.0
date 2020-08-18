#include "ros_pickplace.h"
#define COUT
PickPlaceService::PickPlaceService(ros::NodeHandle n)
{
    this->n_pick = n;
    pickplacePtr = new PickPlace();
}

PickPlaceService::~PickPlaceService()
{
    pickplacePtr = NULL;
    delete pickplacePtr;
}

int PickPlaceService::start()
{

    // n_pick.param("/pickplace_bridge/generator_config_path",  generator_config_path_,
    //              std::string("/home/fshs/work/hirop/config/ClassicGenConfig.yaml"));
    // n_pick.param("/pickplace_bridge/actuator_config_path",   actuator_config_path_,
    //              std::string("/home/fshs/work/hirop/config/ClassicPPConfig.yaml"));

    // n_pick.param("/pickplace_bridge/move_group_name", arm, std::string("arm1"));

    n_pick.param("generator_config_path",  generator_config_path_,
                 std::string("/home/fshs/work/hirop/config/ClassicGenConfig.yaml"));
    n_pick.param("actuator_config_path",   actuator_config_path_,
                 std::string("/home/fshs/work/hirop/config/ClassicPPConfig.yaml"));

    n_pick.param("move_group_name", arm, std::string("arm1"));

    n_pick.param("open_gripper_server_name", openGripperServerName, std::string("openGripper"));
    n_pick.param("close_gripper_server_name", closeGripperServerName, std::string("closeGripper"));

    ROS_INFO_STREAM(arm << "<<----------------");

    ROS_INFO("generator_config_path_:%s", generator_config_path_.c_str());
    ROS_INFO("actuator_config_path_:%s", actuator_config_path_.c_str());

    set_genActuator = n_pick.advertiseService("setGenActuator", &PickPlaceService::setGenActuatorCB, this);
    list_generator = n_pick.advertiseService("listGenerator", &PickPlaceService::listGeneratorCB, this);
    list_actuator = n_pick.advertiseService("listActuator", &PickPlaceService::listActuatorCB, this);
    show_objct = n_pick.advertiseService("showObject", &PickPlaceService::showObjectCB, this);
    remove_object = n_pick.advertiseService("removeObject", &PickPlaceService::removeObjectCB, this);
    pick_server = n_pick.advertiseService("pick", &PickPlaceService::pickCB, this);
    place_server = n_pick.advertiseService("place", &PickPlaceService::placeCB, this);
    move_to_pos = n_pick.advertiseService("moveToPos", &PickPlaceService::moveToPosCB, this);
    move_to_name = n_pick.advertiseService("moveToFoName", &PickPlaceService::moveToNameCB, this);
    pickplace_stop = n_pick.advertiseService("pickplacStop", &PickPlaceService::pickplaceStopCB, this);
    //默认优先启动
    initGenAndActParam("ClassicGenerator", generator_config_path_, "ClassicActuator", actuator_config_path_);

    /****/
    setPickDataServer = n_pick.advertiseService("loadPickData", &PickPlaceService::setPickDataCB, this);
    setPlaceDataServer = n_pick.advertiseService("loadPlaceData", &PickPlaceService::setPlaceDataCB, this);
    setVelocityAcceleratedServer = n_pick.advertiseService("setVelocityAccelerated", &PickPlaceService::setVelocityAcceleratedCB, this);


    openGripperClient = n_pick.serviceClient<hirop_msgs::openGripper>(openGripperServerName);
    closeGripperClient = n_pick.serviceClient<hirop_msgs::closeGripper>(closeGripperServerName);

    pickTraSub = n_pick.subscribe<moveit_msgs::PickupActionResult>("/pickup/result", 10, &PickPlaceService::pickTraCB, this);
    placeTraSub = n_pick.subscribe<moveit_msgs::PlaceActionResult>("/place/result", 10, &PickPlaceService::placeTraCB, this);
    _moveGroup = new moveit::planning_interface::MoveGroupInterface(arm);
    /****/

    return 0;
}

bool PickPlaceService::initGenAndActParam(std::string generatorName, std::string generator_config_path_,std::string actuatorName,std::string actuator_config_path_)
{
    bool isSucceeful = false;
    int ret = this->pickplacePtr->setGenerator(generatorName, "0", generator_config_path_);
    if(ret != 0){
        isSucceeful = false;
        return isSucceeful;
    }
    if(this->pickplacePtr->setActuator(actuatorName, "0", actuator_config_path_) != 0){
        isSucceeful = false;
        return isSucceeful;
    }
    this->pickplacePtr->setMoveGroup(arm);
   isSucceeful = true;
   return isSucceeful;
}

bool PickPlaceService::setGenActuatorCB(hirop_msgs::SetGenActuator::Request &req, hirop_msgs::SetGenActuator::Response &res)
{
    std::string generatorName = req.generatorName;
    std::string actuatorName = req.actuatorName;
    std::string gen_configFile = generator_config_path_;
    std::string act_configFile = actuator_config_path_;

#ifdef COUT
    ROS_INFO("generatorName:%s", generatorName.c_str());
    ROS_INFO("actuatorName:%s", actuatorName.c_str());
    ROS_INFO("generator_config_path_:%s", gen_configFile.c_str());
    ROS_INFO("actuator_config_path_:%s", act_configFile.c_str());
#endif
/*
    int ret = this->pickplacePtr->setGenerator(generatorName, "0", gen_configFile);
    if(ret != 0){
        res.isSucceeful = false;
        return false;
    }
    if(this->pickplacePtr->setActuator(actuatorName, "0", act_configFile) != 0){
        res.isSucceeful = false;
        return false;
    }
    res.isSucceeful = true;

*/
    res.isSucceeful = initGenAndActParam(generatorName, gen_configFile, actuatorName, act_configFile);
    this->pickplacePtr->setMoveGroup(arm);
    return res.isSucceeful;
}

bool PickPlaceService::listGeneratorCB(hirop_msgs::listGenerator::Request &req, hirop_msgs::listGenerator::Response &res)
{
    std::vector<std::string> nameList;
    this->pickplacePtr->getGeneratorList(nameList);
    for(int i=0; i<nameList.size(); i++){
        res.generatorList.push_back(nameList[i]);
    }
    return true;
}

bool PickPlaceService::listActuatorCB(hirop_msgs::listActuator::Request &req, hirop_msgs::listActuator::Response &res)
{
    std::vector<string> nameList;
    this->pickplacePtr->getActuatorList(nameList);
    for(int i =0; i<nameList.size(); i++){
        res.actuatorList.push_back(nameList[i]);
    }
    return true;
}

bool PickPlaceService::showObjectCB(hirop_msgs::ShowObject::Request &req, hirop_msgs::ShowObject::Response &res)
{
    PoseStamped objPos;
    objPos.frame_id = req.objPose.header.frame_id;
    objPos.pose.position.x = req.objPose.pose.position.x;
    objPos.pose.position.y = req.objPose.pose.position.y;
    objPos.pose.position.z = req.objPose.pose.position.z;
    objPos.pose.orientation.w = req.objPose.pose.orientation.w;
    objPos.pose.orientation.x = req.objPose.pose.orientation.x;
    objPos.pose.orientation.y = req.objPose.pose.orientation.y;
    objPos.pose.orientation.z = req.objPose.pose.orientation.z;
    if(this->pickplacePtr->showObject(objPos) != 0){
        res.isSetFinsh = false;
        return false;
    }
    res.isSetFinsh = true;
    return true;
}

bool PickPlaceService::removeObjectCB(hirop_msgs::RemoveObject::Request &req, hirop_msgs::RemoveObject::Response &res)
{
    if(this->pickplacePtr->removeObject() != 0){
        res.isSetFinsh = false;
        return false;
    }
    res.isSetFinsh = true;
    return true;
}

bool PickPlaceService::moveToPosCB(hirop_msgs::MoveToPos::Request &req, hirop_msgs::MoveToPos::Response &res)
{
    PoseStamped movePos;
    movePos.frame_id = req.movePos.header.frame_id;
    movePos.pose.position.x = req.movePos.pose.position.x;
    movePos.pose.position.y = req.movePos.pose.position.y;
    movePos.pose.position.z = req.movePos.pose.position.z;
    movePos.pose.orientation.w = req.movePos.pose.orientation.w;
    movePos.pose.orientation.x = req.movePos.pose.orientation.x;
    movePos.pose.orientation.y = req.movePos.pose.orientation.y;
    movePos.pose.orientation.z = req.movePos.pose.orientation.z;

    if(this->pickplacePtr->moveToPos(movePos) != 0){
        res.isFinsh = false;
        return false;
    }
    res.isFinsh = true;
    return true;
}

bool PickPlaceService::moveToNameCB(hirop_msgs::MoveToName::Request &req, hirop_msgs::MoveToName::Response &res)
{
    if(this->pickplacePtr->moveToFoName(req.PosName) != 0){
        res.isFinsh = false;
        return false;
    }
    res.isFinsh = true;
    return true;
}

bool PickPlaceService::pickCB(hirop_msgs::Pick::Request &req, hirop_msgs::Pick::Response &res)
{
    PoseStamped pickPose;

    pickPose.frame_id = req.pickPos.header.frame_id;
    pickPose.pose.position.x = req.pickPos.pose.position.x;
    pickPose.pose.position.y = req.pickPos.pose.position.y;
    pickPose.pose.position.z = req.pickPos.pose.position.z;
    pickPose.pose.orientation.w = req.pickPos.pose.orientation.w;
    pickPose.pose.orientation.x = req.pickPos.pose.orientation.x;
    pickPose.pose.orientation.y = req.pickPos.pose.orientation.y;
    pickPose.pose.orientation.z = req.pickPos.pose.orientation.z;


    // this->pickplacePtr->setPickPose(pickPose);
    this->pickplacePtr->setPickPos(pickPose);
    if(this->pickplacePtr->pick() != 0){
        res.isPickFinsh = false;
        return false;
    }
    else
    {
        int cnt = 0;
        while (ros::ok() && cnt < 10)
        {
            if(receivePickTra)
            {
                std::vector<moveit_msgs::RobotTrajectory> tra;
                tra = adjustmentVelocity(Picktraject);
                receivePickTra = false;
                pick(tra);
                res.isPickFinsh = true;
                return true;
            }
            ros::Duration(0.5).sleep();
            cnt ++;
        }    
    }
    return true;
}

bool PickPlaceService::placeCB(hirop_msgs::Place::Request &req, hirop_msgs::Place::Response &res)
{

    PoseStamped placePose;

    placePose.frame_id = req.placePos.header.frame_id;
    placePose.pose.position.x = req.placePos.pose.position.x;
    placePose.pose.position.y = req.placePos.pose.position.y;
    placePose.pose.position.z = req.placePos.pose.position.z;
    placePose.pose.orientation.w = req.placePos.pose.orientation.w;
    placePose.pose.orientation.x = req.placePos.pose.orientation.x;
    placePose.pose.orientation.y = req.placePos.pose.orientation.y;
    placePose.pose.orientation.z = req.placePos.pose.orientation.z;

    // this->pickplacePtr->setPlacePose(placePose);
    this->pickplacePtr->setPlacePos(placePose);
    if(this->pickplacePtr->place() != 0){
        res.isPlaceFinsh = false;
        return false;
    }
    else
    {
        int cnt = 0;
        while (ros::ok() && cnt < 10)
        {
            if(receivePlaceTra)
            {
                receivePlaceTra = false;
                std::vector<moveit_msgs::RobotTrajectory> tra;
                tra = adjustmentVelocity(Placetraject);
                place(tra);
                res.isPlaceFinsh = true;
                return true;
            }
            ros::Duration(0.5).sleep();
            cnt ++;
        }    
    }
    return true;
}

bool PickPlaceService::pickplaceStopCB(hirop_msgs::PickPlaceStop::Request &req, hirop_msgs::PickPlaceStop::Response &res)
{
    if(this->pickplacePtr->stop() != 0){
        res.isSucceed = false;
        return false;
    }
    res.isSucceed = true;
    return true;
}

std::vector<moveit_msgs::RobotTrajectory> PickPlaceService::adjustmentVelocity(std::vector<moveit_msgs::RobotTrajectory> traject)
{
    robot_trajectory::RobotTrajectory rt(_moveGroup->getCurrentState()->getRobotModel(), _moveGroup->getName());
    std::vector<moveit_msgs::RobotTrajectory> tras;
    tras.resize(traject.size());
    ROS_INFO_STREAM(traject.size());
    for(int i=0; i<traject.size(); i++)
    {
        moveit_msgs::RobotTrajectory tra;
        tra = adjustmentPoints(traject[i]);
        rt.setRobotTrajectoryMsg(*(_moveGroup->getCurrentState()), tra);
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        ROS_INFO_STREAM("velocity: " <<velocity << " " << "accelerated: " << accelerated);
        iptp.computeTimeStamps(rt, velocity, accelerated);
        rt.getRobotTrajectoryMsg(tras[i]);
    }
    return tras;
}

moveit_msgs::RobotTrajectory PickPlaceService::adjustmentPoints(moveit_msgs::RobotTrajectory& traject)
{
    moveit_msgs::RobotTrajectory tra;
    tra.joint_trajectory.joint_names = traject.joint_trajectory.joint_names;
    tra.joint_trajectory.header.frame_id = traject.joint_trajectory.header.frame_id;
    for(int i=0; i<traject.joint_trajectory.points.size(); i++)
    {
        if(i%4 != 0 && i != traject.joint_trajectory.points.size() - 1) 
            continue;
        tra.joint_trajectory.points.push_back(traject.joint_trajectory.points[i]);
    }
    return tra;
}

void PickPlaceService::setVelocityAccelerated(double v, double a)
{
    velocity = v;
    accelerated = a;
}

bool PickPlaceService::execute(moveit_msgs::RobotTrajectory& tra)
{
    moveit::planning_interface::MoveGroupInterface::Plan multi_plan;
    multi_plan.trajectory_ = tra;
    bool flag = false;
    int cnt = 0;
    while (cnt < 3 && !flag)
    {
        flag = _moveGroup->execute(multi_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        cnt++;
    }
    return flag;
}

bool PickPlaceService::setVelocityAcceleratedCB(hirop_msgs::setVelocityAccelerated::Request& req, hirop_msgs::setVelocityAccelerated::Response& rep)
{
    setVelocityAccelerated(req.Velocity, req.Accelerated);
    return true;
}

bool PickPlaceService::setPickDataCB(hirop_msgs::pickPlaceData::Request& req, hirop_msgs::pickPlaceData::Response& rep)
{
    std::string path = ros::package::getPath("pickplace_bridge");
    std::string filePath = path + "/config/" + req.fileName + ".yaml";
    ROS_INFO_STREAM(filePath);
    this->pickplacePtr->getPickDataNode(filePath);
    return true;
}

bool PickPlaceService::setPlaceDataCB(hirop_msgs::pickPlaceData::Request& req, hirop_msgs::pickPlaceData::Response& rep)
{
    std::string path = ros::package::getPath("pickplace_bridge");
    std::string filePath = path + "/config/" + req.fileName + ".yaml";
    ROS_INFO_STREAM(filePath);
    this->pickplacePtr->getPlaceDataNode(filePath);
    return true;
}


void PickPlaceService::pickTraCB(const moveit_msgs::PickupActionResultConstPtr& msg)
{
    std::vector<moveit_msgs::RobotTrajectory>().swap(Picktraject);
    std::vector<std::string> jointNames = _moveGroup->getJointNames();
    for(int i=0; i<msg->result.trajectory_stages.size(); i++)
    {
        if(jointNames[0] != msg->result.trajectory_stages[i].joint_trajectory.joint_names[0])
            continue;
        Picktraject.push_back(msg->result.trajectory_stages[i]);
    }
    receivePickTra = true;
}

void PickPlaceService::placeTraCB(const moveit_msgs::PlaceActionResultConstPtr& msg)
{
    std::vector<moveit_msgs::RobotTrajectory>().swap(Placetraject);
    std::vector<std::string> jointNames = _moveGroup->getJointNames();
    for(int i=0; i<msg->result.trajectory_stages.size(); i++)
    {
        if(jointNames[0] != msg->result.trajectory_stages[i].joint_trajectory.joint_names[0])
            continue;
        Placetraject.push_back(msg->result.trajectory_stages[i]);
    }
    receivePlaceTra = true;
}


bool PickPlaceService::openGripper()
{
    hirop_msgs::openGripper srv;
    if(openGripperClient.call(srv))
    {
        return srv.response.isOpen;
    }
    else
    {
        return false;
    }
}

bool PickPlaceService::closeGripper()
{
    hirop_msgs::closeGripper srv;
    if(closeGripperClient.call(srv))
    {
        return srv.response.isClose;
    }
    else
    {
        return false;
    }
}


int PickPlaceService::pick(std::vector<moveit_msgs::RobotTrajectory> tra)
{
    if(execute(tra[0]))
    {
        openGripper();
        if(execute(tra[1]))
        {
            closeGripper();
            _moveGroup->attachObject("object");
            if(execute(tra[2]))
            {
                ROS_INFO("pick over");
                return 0;
            }
        }
    }
    ROS_INFO("pick failed");
    return -1;
}

int PickPlaceService::place(std::vector<moveit_msgs::RobotTrajectory> tra)
{
    if(execute(tra[0]))
    {
        if(execute(tra[1]))
        {
            openGripper();
            if(execute(tra[2]))
            {
                // _moveGroup->detachObject("object");
                this->pickplacePtr->removeObject();
                ROS_INFO("place over");
                return 0;
            }
        }
    }
    ROS_INFO("place failed");
    return -1;
}

