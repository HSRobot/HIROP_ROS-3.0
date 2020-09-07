#ifndef HSCFSM_BRIDGE_VOICECTLROBROSFUNC_H
#define HSCFSM_BRIDGE_VOICECTLROBROSFUNC_H

#include <HsTaskFramework.h>
#include "ros/ros.h"
#include <iostream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <hirop_msgs/setStartUpProject.h>
#include <hirop_msgs/setStopProject.h>
//消息类型头文件
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "rb_msgAndSrv/rb_DoubleBool.h"
#include "rb_msgAndSrv/rb_string.h"
#include "sensor_msgs/Image.h"
// #include <cv_bridge/cv_bridge.h>
#include "hsr_rosi_device/ClearFaultSrv.h"
#include "hsr_rosi_device/SetEnableSrv.h"
#include "industrial_msgs/RobotStatus.h"
#include "geometry_msgs/Wrench.h"
#include "hsr_rosi_device/setModeSrv.h"
#include "hirop_msgs/robotError.h"
#include "hirop_msgs/detection.h"
#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/shakeHandSet.h"
#include "hirop_msgs/shakeHandStatus.h"
#include "industrial_msgs/StopMotion.h"
#include "rb_msgAndSrv/rb_EmptyAndInt.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/SetBool.h"
#include <moveit_msgs/RobotTrajectory.h>

// 数据管理
#include "hirop_msgs/loadJointsData.h"
#include "hirop_msgs/loadPoseData.h"
// motion
#include "hirop_msgs/moveSigleAixs.h"
#include "hirop_msgs/moveLine.h"
#include "hirop_msgs/motionBridgeStart.h"
// 轴单
#include "hirop_msgs/moveToSiglePose.h"
// 轴多
#include "hirop_msgs/moveToMultiPose.h"
#include "hirop_msgs/dualRbtraject.h"

#include "hirop_msgs/addJointPose.h"
#include "hirop_msgs/addPose.h"
#include "hirop_msgs/getTrajectory.h"

#include "hirop_msgs/incrementAngle.h"
#include "hirop_msgs/detection.h"
#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/moveSeqIndex.h"
#include "hirop_msgs/Pick.h"
#include "hirop_msgs/Place.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "hirop_msgs/incrementAngle.h"
#include "hirop_msgs/detection.h"
#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/moveSeqIndex.h"

struct StateMonitor{
    bool RobNormalState= false;//机器人正常信号
    bool RobEnableState= false;//机器人伺服信号
    bool flag_rbCtlBusy= false;//机器人空闲信号
    bool isEnd_shakeHand= false;//握手结束信号
    int voice_order= -1;//语音指令码信号
    int hasPeople=0;//行人检测信号
    bool grab_ok=false;//抓取娃娃完成
    bool isOk_robPreparePose= false; //去到握手等待点完成
    geometry_msgs::PoseStamped  object_pose;//yolo6D目标点
    bool detectOk= false;

};

struct rosTopicHandle{
    ros::ServiceClient RobReset_client;
    ros::ServiceClient RobEnable_client;
    ros::ServiceClient handClaw_gesture_client;
    ros::ServiceClient handClaw_shakeHand_client;
    ros::ServiceClient handClaw_grabDoll_client;
    ros::ServiceClient rob_goHome_client;
    ros::ServiceClient RobSetMode_client;
    ros::ServiceClient robGetStatus_client;
    ros::ServiceClient personDetect_client;
    ros::ServiceClient switch_personDetect_client;
    ros::ServiceClient switch_voiceDetect_client;
    ros::ServiceClient backHomeClient ;
    ros::ServiceClient detectePointClient;
    ros::ServiceClient stopMotionClient;
    ros::ServiceClient detectionClient;
    ros::ServiceClient sayGoodByeAction_client;
    ros::ServiceClient robOKAction_client;
    ros::ServiceClient StartImpedence_client;
    ros::ServiceClient CloseImpedence_client;
    ros::ServiceClient startShakeHandJudge_client;
    ros::ServiceClient closeShakeHandJudge_client;

    ros::ServiceClient setByeByeAction_client;
    ros::ServiceClient setByeByeInterrupt_client;
    ros::ServiceClient pickServer_client;
    ros::ServiceClient placeServer_client;

    // motion
    ros::ServiceClient jointMultiClient;
    ros::ServiceClient exeTrajectoryClinet;
    // data_manager
    ros::ServiceClient loadPoseDataClient;
    ros::ServiceClient loadJointDataClient;
    // planner
    ros::ServiceClient jointPlannerClinet;
    ros::ServiceClient cartesianPlannerClient;
    ros::ServiceClient getTrajectoryClinet;

    ros::Publisher voice_order_publisher;
    ros::Publisher visionDetech_publisher;
    ros::Publisher rbGoHome_publisher;
    ros::Publisher flag_forceSensor_publisher;
    ros::Publisher impedenceLive_publisher;
    ros::Publisher shakehandOver_publisher;
    ros::Publisher robStatusSend_publisher;
    ros::Publisher robSpeedSet_publisher;

    ros::Subscriber rbCtlBusy_subscriber;
    ros::Subscriber robStatus_subscriber;
    ros::Subscriber isOpenFollow_subscriber;
    ros::Subscriber getShakeHandResult_subscriber;
    ros::Subscriber objectArraySub ;
    ros::Subscriber voice_order_sub ;
    ros::Subscriber people_detect_sub ;
    ros::Subscriber shakehandstatus_sub ;

};

class VoiceCtlRobRosFunc{
public:
    explicit VoiceCtlRobRosFunc(ros::NodeHandle *node);
    ~VoiceCtlRobRosFunc();
private:
    ros::NodeHandle *Node;
    rosTopicHandle rosTopicHd;
    StateMonitor statemonitor;

    std::string homeFile;
    std::string handgestureFile;
    std::string OkPOseFile;
    std::string palceObjFile;

    std::vector<std::vector<double> > HomeJointsPose;
    std::vector<geometry_msgs::PoseStamped> handgesturePose;
    std::vector<geometry_msgs::PoseStamped> OKPose;

    ros::NodeHandle nh;
    // motion
    ros::ServiceClient jointSigleClient;
    ros::ServiceClient jointMultiClient;
    ros::ServiceClient sigleAxisClient;
    // ros::ServiceClient
    // data_manager
    ros::ServiceClient loadPoseDataClient;
    ros::ServiceClient loadJointDataClient;
    // pickplace
    ros::ServiceClient MoveToHomeClient;
    ros::ServiceClient pickPlaceStopClient;
    ros::ServiceClient updateActuatorClient;
    ros::ServiceClient updateGeneratorClient;
    // planner
    ros::ServiceClient setFromIKClient;
    ros::ServiceClient jointPlannerClinet;
    ros::ServiceClient cartesianPlannerClient;
    ros::ServiceClient getTrajectoryClinet;

    /****/
    std::string waveFile;
    std::vector<geometry_msgs::PoseStamped> wavePose;
    ros::ServiceClient sigleAxisPlannerClient;
    ros::ServiceClient fiveFingerSeqClient;
    const int GRASP_INDEX = 2;
    const int OK_INDEX = 3;
    const int HOME_INDEX = 4;
    const int SHAKE_PREPARE_INDEX = 5;
    const int TAKE_PHOTO_INDEX = 6;

public:
    std::string detectionFile;
//    std::vector<geometry_msgs::PoseStamped> detectionPose;
    std::vector<std::vector<double > > detectionPose;
    std::vector<geometry_msgs::PoseStamped> placeObjPose;


    ros::ServiceClient pickClient;
    ros::ServiceClient placeClient;

    // detect
    ros::ServiceClient detectClient;

    ros::Subscriber objSub;
    std::vector<geometry_msgs::PoseStamped> pickObjPoses;
public:
    //获取ros信号状态
    StateMonitor getStateMonitor();

    void initStateMonitor();

    void resetfunction();
    //行人检测开
    void PersonDetect_Switch(bool flag);
    //行人检测关
    void VoiceDetect_Switch(bool flag);
    //去到握手抬起点
    int robGotoShakeHandPose();
    //开启阻抗 (阻塞式)
    int startImpedence();
    //关闭阻抗
    int closeImpedence();
    //机器人去到娃娃拍照点
    int detectPose();
    //机器人执行yolo6d图像检测
    int detect(std::string& detectObj);
    //机器人抓取娃娃
    int pick(geometry_msgs::PoseStamped& pose);
    int place(geometry_msgs::PoseStamped& pose);
    // 回原点
    int RobGoHome();
    //机器人发声
    void RobSayWords(std::string words);
    int wave();

    void set_hasVoiceOder(int value);
private:
    //话题服务对象制造器
    void initRosToptic();

    int loadRobotPose(std::vector<geometry_msgs::PoseStamped>& poses, std::string fileName);
    int loadRobotPose(std::vector<std::vector<double> >& ,std::string fileName);
    int movePose(std::vector<geometry_msgs::PoseStamped>&);
    int movePose(std::vector<std::vector<double> >&);
    int cartisianPlanner(std::vector<geometry_msgs::PoseStamped>& poses);
    int jointSpacePlanner(std::vector<std::vector<double> >&);
    int getTrajectory(moveit_msgs::RobotTrajectory& tra);
    int motionMulti(moveit_msgs::RobotTrajectory& tra);
    void objSubCB(const hirop_msgs::ObjectArrayConstPtr& msg);

    int sigleAxisPlanner(std::vector<double>&, std::vector<std::string>& );
    int setfiveFingerIndex(const int& index);

    int OK();

    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);

    //ros节点回调函数
    void callback_robStatus_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status);
    void callback_rbCtlBusy_status_subscriber(std_msgs::Bool msg);
    void callback_isOpenFollow_subscriber(std_msgs::Bool msg);
    void callback_getShakeResult_subscriber(std_msgs::Int16 msg);

    void callback_objectCallBack(hirop_msgs::ObjectArray obj);
    void voice_order_CallBack(const std_msgs::Int16::ConstPtr & msg);
    void people_detect_CallBack(const std_msgs::Bool::ConstPtr & msg);
    void shakehandstatus_CallBack(const hirop_msgs::shakeHandStatus::ConstPtr msg);
};


#endif //HSCFSM_BRIDGE_VOICECTLROBROSFUNC_H
