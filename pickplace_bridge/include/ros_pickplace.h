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
#include "hirop_msgs/closeGripper.h"
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/setVelocityAccelerated.h"
#include "hirop_msgs/pickPlaceData.h"
#include <moveit_msgs/PickupActionResult.h>
#include <moveit_msgs/PlaceActionResult.h>
#include <ros/package.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
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
    bool showObjectCB(hirop_msgs::ShowObject::Request& req, hirop_msgs::ShowObject::Response& res);
    bool removeObjectCB(hirop_msgs::RemoveObject::Request& req, hirop_msgs::RemoveObject::Response& res);
    bool moveToNameCB(hirop_msgs::MoveToName::Request& req, hirop_msgs::MoveToName::Response& res);
    bool moveToPosCB(hirop_msgs::MoveToPos::Request& req, hirop_msgs::MoveToPos::Response& res);
    bool pickCB(hirop_msgs::Pick::Request& req, hirop_msgs::Pick::Response& res);
    bool placeCB(hirop_msgs::Place::Request& req, hirop_msgs::Place::Response& res);
    bool pickplaceStopCB(hirop_msgs::PickPlaceStop::Request& req, hirop_msgs::PickPlaceStop::Response& res);
    /****/
    // 服务回调
    bool setVelocityAcceleratedCB(hirop_msgs::setVelocityAccelerated::Request& req, hirop_msgs::setVelocityAccelerated::Response& rep);
    bool setPickDataCB(hirop_msgs::pickPlaceData::Request& req, hirop_msgs::pickPlaceData::Response& rep);
    bool setPlaceDataCB(hirop_msgs::pickPlaceData::Request& req, hirop_msgs::pickPlaceData::Response& rep);
    // 话题回调
    void pickTraCB(const moveit_msgs::PickupActionResultConstPtr& msg);
    void placeTraCB(const moveit_msgs::PlaceActionResultConstPtr& msg);

    std::vector<moveit_msgs::RobotTrajectory> adjustmentVelocity(std::vector<moveit_msgs::RobotTrajectory> traject);
    moveit_msgs::RobotTrajectory adjustmentPoints(moveit_msgs::RobotTrajectory& traject);
    void setVelocityAccelerated(double v, double a);
    bool execute(moveit_msgs::RobotTrajectory& tra);
    bool openGripper();
    bool closeGripper();
    int pick(std::vector<moveit_msgs::RobotTrajectory> tra);
    int place(std::vector<moveit_msgs::RobotTrajectory> tra);
    /****/
private:
    bool initGenAndActParam(std::string generator_config_path_, std::string generatorName,std::string actuator_config_path_, std::string actuatorName);
private:
    ros::NodeHandle n_pick;

    hirop_pickplace::PickPlace *pickplacePtr;
    ros::ServiceServer set_genActuator;
    ros::ServiceServer list_generator;
    ros::ServiceServer list_actuator;
    ros::ServiceServer show_objct;
    ros::ServiceServer remove_object;
    ros::ServiceServer move_to_name;
    ros::ServiceServer move_to_pos;
    ros::ServiceServer pick_server;
    ros::ServiceServer place_server;
    ros::ServiceServer pickplace_stop;

    /****/
    ros::ServiceServer setVelocityAcceleratedServer;
    ros::ServiceServer setPickDataServer;
    ros::ServiceServer setPlaceDataServer;

    ros::ServiceClient openGripperClient;
    ros::ServiceClient closeGripperClient;

    ros::Subscriber pickTraSub;
    ros::Subscriber placeTraSub;
    double velocity = 1;
    double accelerated = 1;
    std::vector<moveit_msgs::RobotTrajectory> Picktraject;
    bool receivePickTra = false;
    std::vector<moveit_msgs::RobotTrajectory> Placetraject;
    bool receivePlaceTra = false;
    moveit::planning_interface::MoveGroupInterface* _moveGroup;
    std::string arm;
    std::string openGripperServerName;
    std::string closeGripperServerName;
    /****/
    geometry_msgs::PoseStamped pickPos_;
    geometry_msgs::PoseStamped placePos_;

    std::string generator_config_path_;
    std::string actuator_config_path_;
};
