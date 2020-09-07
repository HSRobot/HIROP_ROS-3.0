
#ifndef FORCE_BRIDGE_FORCESERVICE_H
#define FORCE_BRIDGE_FORCESERVICE_H

#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"
#include "force/forcePluginAggre.h"
#include "matrixUtily.h"
#include "moveit/move_group_interface/move_group_interface.h"
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "vector"
#include "string"
#include "atomic"
#include "memory"
#include <eigen_conversions/eigen_msg.h>
#include <tf/LinearMath/Quaternion.h>
#include "yaml-cpp/yaml.h"
#include <iomanip>
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Int16.h"
#include "industrial_msgs/RobotStatus.h"
#include "hirop_msgs/force_algorithmChange.h"
#include <realtime_tools/realtime_publisher.h>
#include <hirop_msgs/ImpedenceAdjustStart.h>
#include <hirop_msgs/ImpedenceAdjustStop.h>
#include <MotionCoorUlity.h>

using namespace std;

#define PUBPOSE_HZ 39

enum ForceMode{
    Impenderr = 0,
    ForceTechPoint = 1,
    AdjustImpendControl =2,
};

struct MoveGroup{
    moveit::planning_interface::MoveGroupInterface *move_group;
    robot_model::RobotModelConstPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    std::vector<std::string> joint_names;
    std::string groupName;
    string endlinkName;
};

class forceService {
public:
    explicit forceService(ros::NodeHandle* n);
    ~forceService();
    int start();

private:
    forcePluginAggre *force_plugin;
    MoveGroup MG;

    bool isSim ;
    bool XZReverseDirect;

    atomic<bool> is_stop ;
    atomic<bool> is_running ;
    bool flag_SetForceBias;
    atomic<bool> robot_servo_status;
    vector<double > currentForce,bias_force; //当前力传感器数据,力传感器零点偏差
    //yaml参数保存
    string yamlPara_algorithm_name; //算法插件名称
    double yamlPara_MaxVel_x;
    double yamlPara_MaxVel_y;
    double yamlPara_MaxVel_z;
    vector<double > yamlPara_forceScale;//力传感器数据缩放系数
    vector<bool > yamlPara_forceDrection;//力控方向系数
    vector<double > yamlPara_Stiffness;//刚性
    vector<double > yamlPara_Damping;//阻尼
    vector<double > yamlPara_Mass;//质量
    vector<vector<double >> safetyAreaScope; //划定工作区域
    vector<double> startPos;
    ForceMode mode;
    //ros变量
    ros::NodeHandle* Node;
    ros::Publisher Pose_state_pub;
    ros::ServiceServer impedenceStart_server; //纯阻抗
    ros::ServiceServer impedenceClose_server; //纯阻抗


    ros::ServiceServer force_algorithmChange_server;
    ros::Subscriber   force_sub, robot_status_sub;
    typedef boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > RtPublisherPtr;
    RtPublisherPtr joint_state_pub; //joint listen

    boost::shared_ptr<MotionCoorUlity> motionCoorPtr;
    ros::ServiceClient stopMotionSer;
public:

    /**
     * 阻抗控制启动服务
     * @param req
     * @param res
     * @return
     */
    bool impedenceStartCB(hirop_msgs::ImpedenceAdjustStartRequest &req, hirop_msgs::ImpedenceAdjustStartResponse &res);
    /**
     * 阻抗控制关闭服务
     * @param req
     * @param res
     * @return
     */
    bool impedenceCloseCB(hirop_msgs::ImpedenceAdjustStopRequest &req, hirop_msgs::ImpedenceAdjustStopResponse &res);


    /**
     * @brief force_algorithmChangeCB
     * @param req
     * @param res
     * @return
     */
    bool force_algorithmChangeCB(hirop_msgs::force_algorithmChange::Request &req, hirop_msgs::force_algorithmChange::Response &res);


    //接收传感器数据
    /**
     * @brief forceCallbackZX
     * @param msg
     */
    void forceCallbackZX(const geometry_msgs::Wrench::ConstPtr& msg);

    //接收传感器数据 xz方向输入力矩换向
    /**
     * @brief forceCallbackXZ
     * @param msg
     */
    void forceCallbackXZ(const geometry_msgs::Wrench::ConstPtr& msg);

    /**
     * @brief robotStausCallback 接收机器人状态
     * @param msg
     */
    void robotStausCallback(const industrial_msgs::RobotStatusConstPtr& msg);
public:
    /***
     * 初始化参数
     */
    int initParam();

    /***
     * 开启阻抗控制
     * @return
     */
    bool StartImpedenceCtl();

    /***
     * 停止阻抗控制
     * @return
     */
    int StopImpedenceCtl();

    /***
     * 读取本地参数
     * @return
     */
    int readLocalParam();

    /***
     * 更新阻抗参数
     * @return
     */
    int updateParam();


    void forceDataDeal(const vector<double >& original_force,vector<double >& deal_force);

    /***
     * 设置阻抗算法的输入传感器来源
     * @return
     */
    int setForceInputTopic();

    /***
     * 设置阻抗控制的方向
     * @return
     */
    int setForceControlDirect();

    //运动学模型加载初始化
    bool initKinematic();

    //发布关节坐标执行运动
    void publishPose(std::vector<double> &joint_Pose);

    //初始位姿发布
    void publishOnceForRealRb(std::vector<double> &startPos);

    //获取当前位姿
    void getCurrentPose(geometry_msgs::Pose& Pose);



private:
    /**
     * @brief impdenceErrThreadRun 主要的阻抗运算函数
     */
    void impdenceErrThreadRun();

    /**
     * @brief FKComputePose 机器人正解
     * @param joint_Pose
     * @param Pose
     */
    void FKComputePose(const std::vector<double> &joint_Pose, geometry_msgs::Pose& Pose);

    /**
     * @brief IKCompute 机器人逆解
     * @param Pos
     * @param joint_Pose
     * @return
     */
    int IKCompute(const geometry_msgs::Pose& Pos, std::vector<double> &joint_Pose);

    /**
     * @brief computeImpedence 计算阻抗增量
     * @param force
     * @param outBiasJoint
     * @return
     */
    int computeImpedence(std::vector<double> &force, std::vector<double> &outBiasJoint );

    /**
     * @brief computeAjustImpedence 计算顺应点位
     * @param force
     * @param outJoint
     * @param outPose
     * @return
     */
    int computeAdjustImpedence(std::vector<double> &nextPose, std::vector<double> &force , std::vector<double> &outJoint,\
                              geometry_msgs::Pose &outPose);

    /**
     * @brief computeImpedenceAndResult     阻抗计算并且获取结果
     * @param force
     * @param outJoint
     * @param outPose
     * @return
     */
    int computeImpedenceAndResult( std::vector<double> &force , std::vector<double> &outJoint,\
                                   geometry_msgs::Pose &outPose);


    /**
     * @brief debugImpendXa 打印 增量
     * @param XA
     */
    void debugImpendXa(std::vector<double> &XA);
};


#endif //FORCE_BRIDGE_FORCESERVICE_H
