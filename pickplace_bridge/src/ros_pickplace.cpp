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

    n_pick.param("move_group_config_path", move_group_config_path_, 
                std::string("/home/fshs/work/hirop/config/serialPPMoveGroupConfig.yaml"));

    n_pick.param("move_group_name", arm, std::string("arm"));

    ROS_INFO_STREAM(arm << "<<----------------");

    ROS_INFO("generator_config_path_:%s", generator_config_path_.c_str());
    ROS_INFO("actuator_config_path_:%s", actuator_config_path_.c_str());
    ROS_INFO("move_group_config_path:%s", move_group_config_path_.c_str());

    set_genActuator = n_pick.advertiseService("setGenActuator", &PickPlaceService::setGenActuatorCB, this);
    list_generator = n_pick.advertiseService("listGenerator", &PickPlaceService::listGeneratorCB, this);
    list_actuator = n_pick.advertiseService("listActuator", &PickPlaceService::listActuatorCB, this);
    pick_server = n_pick.advertiseService("pick", &PickPlaceService::pickCB, this);
    place_server = n_pick.advertiseService("place", &PickPlaceService::placeCB, this);
    move_to_pos = n_pick.advertiseService("moveToPos", &PickPlaceService::moveToPosCB, this);
    move_to_name = n_pick.advertiseService("moveToFoName", &PickPlaceService::moveToNameCB, this);
    pickplace_stop = n_pick.advertiseService("pickplacStop", &PickPlaceService::pickplaceStopCB, this);
    //默认优先启动
    initGenAndActParam("Generator", generator_config_path_, "Actuator", actuator_config_path_);

    /****/
    updateGeneratorServer = n_pick.advertiseService("updateGeneratorParam", &PickPlaceService::updateGeneratorCB, this);
    updateActuatorServer = n_pick.advertiseService("updateActuatorParam", &PickPlaceService::updateActuatorCB, this);
    setVelocityAcceleratedServer = n_pick.advertiseService("setVelocityAccelerated", &PickPlaceService::setVelocityAcceleratedCB, this);

    showObjClient = n_pick.serviceClient<hirop_msgs::PubObject>("/loadObject");
    // removeObjectClient = n_pick.serviceClient<hirop_msgs::removeObject>("/rmObject");
    /****/
    groupConfig(move_group_config_path_);
    this->pickplacePtr->setMoveGroup(arm);
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
    removeObj();
    PoseStamped pickPose;
    pickPose.frame_id = req.pickPos.header.frame_id;
    pickPose.pose.position.x = req.pickPos.pose.position.x;
    pickPose.pose.position.y = req.pickPos.pose.position.y;
    pickPose.pose.position.z = req.pickPos.pose.position.z;
    pickPose.pose.orientation.w = req.pickPos.pose.orientation.w;
    pickPose.pose.orientation.x = req.pickPos.pose.orientation.x;
    pickPose.pose.orientation.y = req.pickPos.pose.orientation.y;
    pickPose.pose.orientation.z = req.pickPos.pose.orientation.z;


    this->pickplacePtr->setPickPose(pickPose);
    showObj(pickPose);
    if(this->pickplacePtr->pick() != 0){
        res.isPickFinsh = false;
        return false;
    }
    res.isPickFinsh = true;
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

    this->pickplacePtr->setPlacePose(placePose);
    if(this->pickplacePtr->place() != 0){
        removeObj();
        res.isPlaceFinsh = false;
        return false;
    }
    res.isPlaceFinsh = true;
    removeObj();
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



/****/
bool PickPlaceService::updateGeneratorCB(hirop_msgs::updateParam::Request& req, hirop_msgs::updateParam::Response& rep)
{
    std::string path = ros::package::getPath("pickplace_bridge");
    path = path + "/config/" + req.file + ".yaml";
    ROS_INFO_STREAM("update Generator file: " << path);
    this->pickplacePtr->updateGenerator(path);
    rep.result = true;
    return true;
}

bool PickPlaceService::updateActuatorCB(hirop_msgs::updateParam::Request& req, hirop_msgs::updateParam::Response& rep)
{
    std::string path = ros::package::getPath("pickplace_bridge");
    path = path + "/config/" + req.file + ".yaml";
    ROS_INFO_STREAM("update Actuator file: " << path);
    this->pickplacePtr->updateActuator(path);
    rep.result = true;
    return true;
}

bool PickPlaceService::setVelocityAcceleratedCB(hirop_msgs::setVelocityAccelerated::Request& req, hirop_msgs::setVelocityAccelerated::Response& rep)
{
    this->pickplacePtr->setVelocityAccelerated(req.Velocity, req.Accelerated);
    return true;
}

bool PickPlaceService::showObj(PoseStamped& pose)
{
    hirop_msgs::PubObject show;
    show.request.header.frame_id = pose.frame_id;
    show.request.object_id.push_back("object");
    show.request.primitives.resize(1);
    show.request.primitives[0].type = show.request.primitives[0].BOX;
    show.request.primitives[0].dimensions.resize(3);
    show.request.primitives[0].dimensions[0] = 0.03;
    show.request.primitives[0].dimensions[1] = 0.03;
    show.request.primitives[0].dimensions[2] = 0.06;
    tf::Quaternion quat;
    quat.setW(pose.pose.orientation.w);
    quat.setX(pose.pose.orientation.x);
    quat.setY(pose.pose.orientation.y);
    quat.setZ(pose.pose.orientation.z);
    double R, P, Y;
    tf::Matrix3x3(quat).getRPY(R, P, Y);
    show.request.pose.resize(1);
    show.request.pose[0].position.x = pose.pose.position.x;
    show.request.pose[0].position.y = pose.pose.position.y;
    show.request.pose[0].position.z = pose.pose.position.z;
    show.request.pose[0].rpy.R = R;
    show.request.pose[0].rpy.P = P;
    show.request.pose[0].rpy.Y = Y;
    show.request.rgba.resize(1);
    show.request.rgba[0].r = 100;
    show.request.rgba[0].g = 100;
    show.request.rgba[0].b = 100;
    show.request.rgba[0].a = 1;
    if(showObjClient.call(show))
    {
        ROS_INFO_STREAM("pub object: " << show.response.result);
        return show.response.result;
    }
    ROS_INFO_STREAM("pub object: " << false);
    return false;
}

int PickPlaceService::groupConfig(std::string path)
{
    return this->pickplacePtr->groupConfig(path);
}

bool PickPlaceService::removeObj()
{
    // hirop_msgs::removeObject srv;
    // srv.request.id = "object";
    // if(removeObjectClient.call(srv))
    // {
    //     return srv.response.result;
    // }
    // return false;
}
/****/