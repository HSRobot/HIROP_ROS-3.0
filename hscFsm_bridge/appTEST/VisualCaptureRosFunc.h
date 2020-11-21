#ifndef VISUALCAPTUREROSFUNC_H
#define VISUALCAPTUREROSFUNC_H

#include <HsTaskFramework.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include <pthread.h>

//消息类型头文件
#include <hirop_msgs/detection.h>
#include <hirop_msgs/ObjectArray.h>
#include <hirop_msgs/Pick.h>
#include <hirop_msgs/Place.h>
#include <hsr_rosi_device/ClearFaultSrv.h>
#include <hsr_rosi_device/SetEnableSrv.h>
#include <industrial_msgs/RobotStatus.h>
#include <industrial_msgs/StopMotion.h>

#include <hirop_msgs/loadJointsData.h>
#include <hirop_msgs/loadPoseData.h>

#include <hirop_msgs/moveSigleAixs.h>
#include <hirop_msgs/moveLine.h>
#include <hirop_msgs/motionBridgeStart.h>
#include <hirop_msgs/moveToSiglePose.h>
#include <hirop_msgs/moveToMultiPose.h>
#include <hirop_msgs/dualRbtraject.h>

#include <hirop_msgs/addJointPose.h>
#include <hirop_msgs/addPose.h>
#include <hirop_msgs/getTrajectory.h>

#include <hirop_msgs/LookPointCloud.h>
#include <hirop_msgs/UpdatePCL.h>

#include <hirop_msgs/moveLine.h>


#include <std_msgs/Bool.h>
#include <hirop_msgs/setVelocityAccelerated.h>
#include <hirop_msgs/updateParam.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>

using namespace std;

//状态检测信号数据结构
struct VC_StateMonitor{
    bool RobotStatus;
    bool RobotEnableStatus;
    bool havePickPose;
};

class VisualCaptureRosFunc{
public:

    explicit VisualCaptureRosFunc(ros::NodeHandle *node);
    ~VisualCaptureRosFunc();
    
    void RobotReset();
    void RobotEnable(bool value);
    void RobotStopMotion();
    // int detect(stringVisualCaptureRosFunc:: & object_name);
    int detect(string& object_name);
    int RobotGoHome();
    int RobotGoDetection(int where = 1);
    int RobotPick(geometry_msgs::PoseStamped & pose);
    int RobotPlace(geometry_msgs::PoseStamped & pose);
    int loadRobotPose(vector<geometry_msgs::PoseStamped> & poses, string fileName);
    int loadRobotPose(vector<vector<double> > & joints, string fileName);
    int lookPointCloud();

    int creatPedsDetection();
    void shutdownPedsDetection();

    bool getPedsDetectionState();
    
    int cartesianMotionLine(double x, double y, double z);

    void StateMonitorInit();
    VC_StateMonitor getStateMonitor();

    vector<geometry_msgs::PoseStamped> PickObjPoses;
    vector<geometry_msgs::PoseStamped> PlaceObjPoses;
    vector<geometry_msgs::PoseStamped> PlaceObjPoses2;
private:

    ros::NodeHandle *Node;

    //set_robot
    ros::ServiceClient RobotResetClient;
    ros::ServiceClient RobotEnableClient;
    //pick&place
    ros::ServiceClient detectClient;
    ros::ServiceClient pickClient;
    ros::ServiceClient placeClient;
    //data_manager
    ros::ServiceClient loadPoseDataClient;
    ros::ServiceClient loadJointDataClient;
    //planner
    ros::ServiceClient jointPlannerClient;
    ros::ServiceClient CartesianPlannerClient;
    ros::ServiceClient getTrajectoryClinet;
    //motion
    ros::ServiceClient jointMultiClient;
    ros::ServiceClient exeTrajectoryClient;
    ros::ServiceClient stopMotionClient;
    ros::ServiceClient motionLineClient;

    ros::ServiceClient setVAClient;
    ros::ServiceClient setParamClient;

    ros::Subscriber RobotStatusSub;
    ros::Subscriber ObjectArraySub;
    ros::Subscriber DetectPhotoSub;
    ros::Subscriber detectHumanSub;

    ros::ServiceClient lookPointCloudClient;

    ros::Publisher shutDownPedsDetectionPub;



    vector<vector<double> > HomePoses;
    // vector<geometry_msgs::PoseStamped> DetectionPoses;
    // vector<geometry_msgs::PoseStamped> DetectionPoses3;

    vector<vector<double>> DetectionPoses;
    vector<vector<double>> DetectionPoses3;

    VC_StateMonitor statemonitor;

    bool isPedsdetetion;


private:

    void RosInit();

    void objectDetectSub_CallBack(const hirop_msgs::ObjectArrayConstPtr &msg);
    void robotStatusSub_CallBack(const industrial_msgs::RobotStatus::ConstPtr robot_statu);
    void detectHuman_Callback(const std_msgs::Bool::ConstPtr& msg);

    bool transformFrame(geometry_msgs::PoseStamped & p, string frame_id);

    /***机器人运动控制函数***/

    int movePose(vector<geometry_msgs::PoseStamped> & poses);
    int movePose(vector<vector<double>> & joints);

    int CartesianPlanner(vector<geometry_msgs::PoseStamped> & poses);
    int JointSpacePlanner(vector<vector<double>> & joints);

    int getTrajectory(moveit_msgs::RobotTrajectory & tra);
    int motionMulti(moveit_msgs::RobotTrajectory & tra);

    static void* bringUpPedsDetetion(void* _this);

    /*********************/

};

#endif //VISUALCAPTUREROSFUNC_H
