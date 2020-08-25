#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "hirop_msgs/SetGenActuator.h"
#include "hirop_msgs/ShowObject.h"
#include "hirop_msgs/RemoveObject.h"
#include "hirop_msgs/Pick.h"
#include "hirop_msgs/Place.h"
#include "hirop_msgs/listActuator.h"
#include "hirop_msgs/listGenerator.h"
#include "hirop_msgs/PickPlaceStop.h"
#include "hirop_msgs/MoveToName.h"
#include "hirop_msgs/MoveToPos.h"

/****/
#include <ros/package.h>
#include <hirop_msgs/updateParam.h>
#include <hirop_msgs/setVelocityAccelerated.h>
#include "hirop_msgs/PubObject.h"
#include "hirop_msgs/removeObject.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
/****/
#include "hirop/pickplace/execute_process.h"
#include "hirop/msgs/posestamped.h"

using namespace hirop;
using namespace hirop_pickplace;

class PickPlaceService{
    
public:
    PickPlaceService(ros::NodeHandle n);

    ~PickPlaceService();
    
    int start();

    bool setGenActuatorCB(hirop_msgs::SetGenActuator::Request& req, hirop_msgs::SetGenActuator::Response& res);
    bool listGeneratorCB(hirop_msgs::listGenerator::Request& req, hirop_msgs::listGenerator::Response& res);
    bool listActuatorCB(hirop_msgs::listActuator::Request& req, hirop_msgs::listActuator::Response& res);
    bool moveToNameCB(hirop_msgs::MoveToName::Request& req, hirop_msgs::MoveToName::Response& res);
    bool moveToPosCB(hirop_msgs::MoveToPos::Request& req, hirop_msgs::MoveToPos::Response& res);
    bool pickCB(hirop_msgs::Pick::Request& req, hirop_msgs::Pick::Response& res);
    bool placeCB(hirop_msgs::Place::Request& req, hirop_msgs::Place::Response& res);
    bool pickplaceStopCB(hirop_msgs::PickPlaceStop::Request& req, hirop_msgs::PickPlaceStop::Response& res);

    /****/
    bool updateGeneratorCB(hirop_msgs::updateParam::Request& req, hirop_msgs::updateParam::Response& rep);
    bool updateActuatorCB(hirop_msgs::updateParam::Request& req, hirop_msgs::updateParam::Response& rep);
    bool setVelocityAcceleratedCB(hirop_msgs::setVelocityAccelerated::Request& req, hirop_msgs::setVelocityAccelerated::Response& rep);

    bool showObj(PoseStamped& pose);
    int groupConfig(std::string path);
    /****/
private:
    bool initGenAndActParam(std::string generator_config_path_, std::string generatorName,std::string actuator_config_path_, std::string actuatorName);
private:
    ros::NodeHandle n_pick;

    hirop_pickplace::PickPlace *pickplacePtr;
    ros::ServiceServer set_genActuator;
    ros::ServiceServer list_generator;
    ros::ServiceServer list_actuator;
    ros::ServiceServer move_to_name;
    ros::ServiceServer move_to_pos;
    ros::ServiceServer pick_server;
    ros::ServiceServer place_server;
    ros::ServiceServer pickplace_stop;

    /****/
    ros::ServiceServer updateGeneratorServer;
    ros::ServiceServer updateActuatorServer;
    ros::ServiceServer setVelocityAcceleratedServer;

    ros::ServiceClient showObjClient;
    /****/
    std::string generator_config_path_;
    std::string actuator_config_path_;
    std::string move_group_config_path_;
};
