#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "ros/ros.h"
#include <string>
#include "yaml-cpp/yaml.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "Eigen/Eigen"
#include "sensor_msgs/PointCloud2.h"
#include <hirop_msgs/LookPointCloud.h>
#include <hirop_msgs/LoadPCL.h>
#include "hirop_msgs/SavePCL.h"
#include "hirop_msgs/CleanPCL.h"
#include "hirop_msgs/UploadScene.h"
#include <hirop_msgs/ClearScene.h>
#include <hirop_msgs/UpdatePCL.h>
#include <std_srvs/Empty.h>

/****/
#include "planning_scene_builder.h"
#include "hirop_msgs/PubMesh.h"
#include "hirop_msgs/PubObject.h"
#include "hirop_msgs/removeObject.h"
#include "tf/transform_listener.h"
#include "hirop_msgs/transformFramePose.h"
/****/

using namespace std;

class pereptionFun
{
public:
    explicit pereptionFun(ros::NodeHandle& n);
    void start();

public:
    /**
     * @brief transformFrame  from camera frame  to the world frame
     */
    bool lookPointCloudCB(hirop_msgs::LookPointCloud::Request &req,
                          hirop_msgs::LookPointCloud::Response &res);
    bool clearSceneCB(hirop_msgs::ClearScene::Request &req,
                      hirop_msgs::ClearScene::Response &res);
    bool loadPointCloudCB(hirop_msgs::LoadPCL::Request &req,
                          hirop_msgs::LoadPCL::Response &res);
    bool savePointCloudCB(hirop_msgs::SavePCL::Request &req,
                          hirop_msgs::SavePCL::Response &res);
	bool updatePointCloudCB(hirop_msgs::UpdatePCL::Request &res,
							hirop_msgs::UpdatePCL::Response &req);
    bool transformPoseCB(hirop_msgs::transformFramePose::Request& req,
                            hirop_msgs::transformFramePose::Response& rep);
    void loadLocalParam(const string & path);
    void loadPCD(const string & path);
    void savePCD(const string & path="./");
    int  process();
    void printInfo();
private:
    void initRosTopic();
    void publishCloud();
    void clearPoint();
    void pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    bool transformFrame(const geometry_msgs::PoseStamped &p, geometry_msgs::PoseStamped &target,const string &frame_id);
private:
    ros::NodeHandle node;
    ros::Subscriber receivePclCloudSub, pointCloudSub;
    ros::Publisher publishPclCloudPub;
    ros::Publisher publishOctomapPub;
    ros::ServiceServer lookServer, loadPCServer,savePCServer;
    ros::ServiceServer clearServer,updateServer;
    ros::ServiceServer transformPoseServer;
    
    ros::ServiceClient moveit_clear_octomap_client, lookClient, clearClient;

    /****/
    ros::ServiceServer pubMeshServer;
    bool pubMeshServerCB(hirop_msgs::PubMesh::Request& req, hirop_msgs::PubMesh::Response& rep);
    ros::ServiceServer pubObjectServer;
    bool pubObjectServerCB(hirop_msgs::PubObject::Request& req, hirop_msgs::PubObject::Response& rep);
    ros::ServiceServer rmObject;
    bool rmObjectCB(hirop_msgs::removeObject::Request& req, hirop_msgs::removeObject::Response& rep);
    PlanningSceneBuilder* planningScenePtr;
    /****/
private:
    string pclReceviTopicName;
    string publishPclName;
    string referFrame, baseFrame,cameraFrame;
    YAML::Node yamlNode;

    Eigen::Vector3d T;
    Eigen::Matrix3d R;
    boost::atomic<bool> havePointCloud;
};
