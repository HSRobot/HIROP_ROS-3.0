#include "forceService.h"
#include <boost/make_shared.hpp>
#include <industrial_msgs/StopMotion.h>

#include <chrono>
#include <thread>
#define REAL_FORCE 1
#define PUBPOSE_HZ 40

forceService::forceService(ros::NodeHandle* n):mode(Impenderr) {
    Node = n;
    force_plugin = new forcePluginAggre();
    motionCoorPtr  = boost::make_shared<MotionCoorUlity>();
    is_running = false;

    fakeSensor = boost::make_shared<FakeForceSensor>();
}

forceService::~forceService() {
    if(force_plugin!= nullptr)
    {
        delete force_plugin;
        force_plugin = nullptr;
    }
}

int forceService::start() {
    //参数初始化
    if(initParam()!=0){
        return -1;
    }
    //moveit模型初始化
    if(!initKinematic()){
        return -2;
    }
    return 0;
}

//初始化参数
int forceService::initParam() {
    int ret = 0 ;
    Node->param("/force_bridge/group_name",MG.groupName, string("arm"));
    Node->param("/force_bridge/endlinkName",MG.endlinkName,string("link6"));
    Node->param("/force_bridge/isSim",isSim,false);
    Node->param("/force_bridge/XZReverseDirect",XZReverseDirect,false);
    ROS_INFO_STREAM("groupName "<<MG.groupName);
    ROS_INFO_STREAM("endlinkName "<<MG.endlinkName);
    ROS_INFO_STREAM("isSim "<<isSim);
    ROS_INFO_STREAM("XZReverseDirect: "<<XZReverseDirect);
    if(readLocalParam()!=0){
        return -1;
    }

    //变量初始化
    flag_SetForceBias= false;
    bias_force.resize(6);
    std::fill(bias_force.begin(), bias_force.begin()+6 ,0);
    currentForce.resize(6);
    std::fill(currentForce.begin(), currentForce.begin()+6 ,0);
    //话题初始化
    Pose_state_pub = Node->advertise<geometry_msgs::Pose>("force_bridge/robotPose", 1);
    robot_status_sub=Node->subscribe<industrial_msgs::RobotStatus>("robot_status",1,boost::bind(&forceService::robotStausCallback,this,_1));

    if(isSim)    
        joint_state_pub = boost::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::JointState>>(*Node, "robot_status", 1);
    else
        joint_state_pub = boost::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::JointState>>(*Node, "impedance_err", 1);

    if(!XZReverseDirect)
        force_sub = Node->subscribe("/daq_data", 1, &forceService::forceCallbackXZ, this);
    else
        force_sub = Node->subscribe("/daq_data", 1, &forceService::forceCallbackZX, this);

    force_algorithmChange_server = Node->advertiseService("force_bridge/force_algorithm", &forceService::force_algorithmChangeCB,this);
    impedenceStart_server = Node->advertiseService("force_bridge/impedenceStart", &forceService::impedenceStartCB,this);
    impedenceClose_server = Node->advertiseService("force_bridge/impedenceStop", &forceService::impedenceCloseCB,this);

    stopMotionSer = Node->serviceClient<industrial_msgs::StopMotion>("/stop_motion");


    ROS_INFO_STREAM("--------initParam over------------");

    return ret;

}

//开启阻抗控制
bool forceService::StartImpedenceCtl() {
    ROS_INFO_STREAM("impedanceStartControl strating ! ");
    is_running=true;
    is_stop= false;
    //阻抗插件初始化
    cout<<"algorithm_name: "<<yamlPara_algorithm_name<<endl;
    if(force_plugin->setForcePlugin(yamlPara_algorithm_name, "1", "")!=0){
        return false;
    }
    //设置阻抗算法参数
    force_plugin->setParameters(&yamlPara_Stiffness[0],&yamlPara_Damping[0],&yamlPara_Mass[0]);
    //循环前发布一次起始关节角
    startPos.clear();
    startPos = MG.move_group->getCurrentJointValues();
    MG.kinematic_state->setJointGroupPositions( MG.joint_model_group, startPos);
    if(isSim)
    {
        robot_servo_status = true;
        publishPose(startPos);
    }
    else
    {
        publishOnceForRealRb(startPos);//使用“impedance_err”接口驱动真机，第一次发送数据格式不同，必须要发。
    }

    return true;
}

void forceService::impdenceErrThreadRun()
{
    while(!robot_servo_status)
    {
        if(is_stop)
        {
            is_running= false;
            return ;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    auto nextTime = std::chrono::system_clock::now();

    while(ros::ok()&&(!is_stop)&&(!ros::isShuttingDown())&&(robot_servo_status))
    {

        vector<double > copy_currentForce=currentForce;
        assert(copy_currentForce.size() >0 );
        vector<double > out_dealForce;
        forceDataDeal(copy_currentForce,out_dealForce);

        std::vector<double> outJoint;
        geometry_msgs::Pose outPose;

        switch(mode)
        {
            case Impenderr:
                if(computeImpedenceAndResult(out_dealForce, outJoint,outPose) != 0){
                    continue;
                }
            break;

            case AdjustImpendControl:
                {
                    vector<double> pathJpose;
                    bool ret = motionCoorPtr->getComputTraj(pathJpose);
                    if((!ret) || computeAdjustImpedence(pathJpose,out_dealForce, outJoint,outPose) < 0)
                    {
                        is_stop = true; //停止
                        force_plugin->setParameters(&yamlPara_Stiffness[0],&yamlPara_Damping[0],&yamlPara_Mass[0]);
                        continue;
                    }
//                    std::cout << "------------pathJpose [0] "<<pathJpose[0]*180/3.14<<std::endl;

                }
            break;

        default:
            {
                ROS_ASSERT(1 == 0);
            }
            break;
        }

        //发布位姿
//        Pose_state_pub.publish(outPose);
        //执行运动
        publishPose(outJoint);

        nextTime = nextTime +std::chrono::milliseconds(PUBPOSE_HZ);
        auto duration_in_ms = std::chrono::duration_cast<chrono::milliseconds>(nextTime.time_since_epoch());

        std::cout << "<------------------------------------- "<<duration_in_ms.count()<<std::endl;

        std:this_thread::sleep_until(nextTime );

    }
    is_running= false;

}

void forceService::FKComputePose(const std::vector<double> &joint_Pose, geometry_msgs::Pose &Pose)
{
    assert(joint_Pose.size() == 6);
    MG.kinematic_state->setJointGroupPositions( MG.joint_model_group, joint_Pose);
    const Eigen::Affine3d &end_effector_state =  MG.kinematic_state->getGlobalLinkTransform( MG.endlinkName);
    tf::poseEigenToMsg(end_effector_state, Pose);
}

int forceService::IKCompute(const geometry_msgs::Pose &current_Pose, std::vector<double> &outJoint)
{

    //4.位姿转关节角
    if(!MG.kinematic_state->setFromIK(MG.joint_model_group, current_Pose, MG.endlinkName, 10, 0.1)){
        ROS_ERROR( "IK ERR " );
        return -1;
    }
    std::vector<double> joint_values;
    // 返回计算后的关节角
    MG.kinematic_state->copyJointGroupPositions(MG.joint_model_group, joint_values);

    outJoint =  std::move(joint_values);

    cout<<"点位执行关节 outJoint : ";
    for (size_t i = 0; i < 6; i++)
    {
        cout<<outJoint[i] *180 /M_PI<<" ";
    }
    std::cout <<endl;
    return 0;
}

//停止阻抗控制
int forceService::StopImpedenceCtl() {
    is_stop=true;
    int time_count=0;
    while (is_running&&(time_count<10)){
        sleep(1);
        time_count++;
    }
    return 0;
}

//读取本地参数
int forceService::readLocalParam() {
    //获取当前包路径
    string package_path = ros::package::getPath("force_bridge");
    string configFile_path=package_path+"/config/forceImp.yaml";
    YAML::Node yaml_node=YAML::LoadFile(configFile_path);
    //参数解析
    yamlPara_forceScale=yaml_node["parameters"]["revForceScale"].as<vector<double >>();
    if (yamlPara_forceScale.size()!= 6 ){
        return -1;
    }
    yamlPara_forceDrection=yaml_node["parameters"]["forceDrectionEnable"].as<vector<bool >>();
    if (yamlPara_forceScale.size()!= 6){
        return -2;
    }
    safetyAreaScope.resize(3);
    safetyAreaScope[0]=yaml_node["parameters"]["Limit_safetyAreaScope_x"].as<vector<double >>();
    safetyAreaScope[1]=yaml_node["parameters"]["Limit_safetyAreaScope_y"].as<vector<double >>();
    safetyAreaScope[2]=yaml_node["parameters"]["Limit_safetyAreaScope_z"].as<vector<double >>();

    yamlPara_Stiffness=yaml_node["parameters"]["Stiffness"].as<vector<double >>();
    yamlPara_Damping=yaml_node["parameters"]["Damping"].as<vector<double >>();
    yamlPara_Mass=yaml_node["parameters"]["Mass"].as<vector<double >>();

    yamlPara_algorithm_name=yaml_node["parameters"]["algorithm_name"].as<string >();
//    cout<<"algorithm_name: "<<yamlPara_algorithm_name<<endl;

     yamlPara_MaxVel_x=yaml_node["parameters"]["MaxVel_x"].as<double >();
     yamlPara_MaxVel_y=yaml_node["parameters"]["MaxVel_y"].as<double >();
     yamlPara_MaxVel_z=yaml_node["parameters"]["MaxVel_z"].as<double >();
    return 0;
}

//更新阻抗参数
int forceService::updateParam() {
    return 0;
}

//设置阻抗算法的输入传感器来源
int forceService::setForceInputTopic() {
    return 0;
}

//设置阻抗控制的方向
int forceService::setForceControlDirect() {

    return 0;
}

bool forceService::initKinematic() {
    try{
        MG.move_group = new moveit::planning_interface::MoveGroupInterface(MG.groupName);
    }catch(std::runtime_error &e){
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    MG.move_group->setStartState(*MG.move_group->getCurrentState());

    vector<double> startPos = MG.move_group->getCurrentJointValues();
    assert(startPos.size() > 0);
    MG.kinematic_model = MG.move_group->getRobotModel();
    MG.kinematic_state = MG.move_group->getCurrentState();
    //kinematic_state->setToDefaultValues();
    MG.joint_model_group = const_cast<robot_state::JointModelGroup*>(MG.kinematic_model->getJointModelGroup(MG.groupName));
    MG.joint_names = MG.joint_model_group->getJointModelNames();

    if(MG.joint_model_group == nullptr)
        return false;
    for (int i = 0; i <MG.joint_names.size(); ++i) {
        ROS_INFO_STREAM(MG.joint_names[i]);
    }
    ROS_INFO_STREAM("initKinematic IS OK");
    return true;
}

bool forceService::impedenceStartCB(hirop_msgs::ImpedenceAdjustStartRequest &req, hirop_msgs::ImpedenceAdjustStartResponse &res) {
    if(is_running == false &&StartImpedenceCtl()){
        robot_status_sub = Node->subscribe<industrial_msgs::RobotStatus>("robot_status",1,boost::bind(&forceService::robotStausCallback,this,_1));

    } else{
        res.result = false;
        return true;
    }
    mode = ForceMode(req.mode);

    if(mode == AdjustImpendControl)
    {
        if( req.trajectPath.points.size() == 0 )
        {
            res.result = false;
            ROS_WARN_STREAM("trajectPath point size is 0 , can't exec motion ");
            return true;
        }
        motionCoorPtr->setTraject(req.trajectPath);
        if(!motionCoorPtr->computeTraj())
        {
            res.result = false;
            return true;
        }
#ifndef REAL_FORCE
        fakeSensor->setTimeLength(motionCoorPtr->getTimeLength());
        fakeSensor->setForceEnable(yamlPara_forceDrection);
#endif
    }
    res.result = true;

    boost::function0<void> f1 = boost::bind(&forceService::impdenceErrThreadRun, this);
    boost::thread t1(f1);
    t1.detach();
    return true;
}

bool forceService::impedenceCloseCB(hirop_msgs::ImpedenceAdjustStopRequest &req, hirop_msgs::ImpedenceAdjustStopResponse &res) {
    is_stop=true;
    while (is_running){
        sleep(1);
    }
    industrial_msgs::StopMotion stopSrv;
    stopMotionSer.call(stopSrv);
    res.result = true;
    return true;
}




void forceService::forceCallbackXZ(const geometry_msgs::Wrench::ConstPtr &msg) {
    currentForce[0] = msg->force.x * yamlPara_forceScale[0] ;
    currentForce[1] = -msg->force.y * yamlPara_forceScale[1];
    currentForce[2] = -msg->force.z * yamlPara_forceScale[2];
    currentForce[3] = -msg->torque.x * yamlPara_forceScale[3];
    currentForce[4] = msg->torque.y * yamlPara_forceScale[4];
    currentForce[5] = -msg->torque.z * yamlPara_forceScale[5];
}

void forceService::forceCallbackZX(const geometry_msgs::Wrench_<allocator<void>>::ConstPtr &msg) {
    currentForce[0] = msg->force.z * yamlPara_forceScale[0];
    currentForce[1] = -msg->force.y * yamlPara_forceScale[1];
    currentForce[2] = msg->force.x * yamlPara_forceScale[2];
    currentForce[3] = 0;
    currentForce[4] = 0;
    currentForce[5] = 0;

}


void forceService::robotStausCallback(const industrial_msgs::RobotStatusConstPtr &msg) {
//    if(msg->in_error.val != 0 || msg->in_motion.val != 0 ){
    if(msg->in_error.val != 0 || msg->drives_powered.val!=1 ){
        robot_servo_status = false;
        std::cout << "msg->in_motion.val : "<< std::to_string(msg->in_motion.val) << " msg->in_error.val: "<<  std::to_string(msg->in_error.val)<<std::endl;
        robot_status_sub.shutdown();
        std::cout << "robot_status_sub is shutdown "<<std::endl;

    }else {
        robot_servo_status = true;
    }
}



void forceService::publishOnceForRealRb(std::vector<double> &startPos) {
    startPos.push_back(0);
    sensor_msgs::JointState sensor_compute_robot_state;
    sensor_compute_robot_state.header.stamp = ros::Time::now();
    sensor_compute_robot_state.name.resize(7);
    sensor_compute_robot_state.position = startPos;
    joint_state_pub->msg_ = sensor_compute_robot_state;
    joint_state_pub->unlockAndPublish();
    ROS_INFO_STREAM( "sensor_compute_robot_state size: " << sensor_compute_robot_state.position.size() );
    matrixUtily::dumpDVec(sensor_compute_robot_state.position, 7,"sensor_compute_robot_state: ");
    startPos.pop_back();

}

void forceService::publishPose(std::vector<double> &joint_Pose) {
    sensor_msgs::JointState compute_robot_state;
    compute_robot_state.name.resize(6);
    compute_robot_state.name = MG.joint_names;

    compute_robot_state.position = joint_Pose;
    //ROS_INFO_STREAM(compute_robot_state);
    //运动点打印
//    array<double, 6> tmp;
//    cout<<"点位执行关节: ";
//    for (size_t i = 0; i < 6; i++)
//    {
//        tmp[i]=compute_robot_state.position[i]*180/M_PI;
//        cout<<tmp[i]<<" ";
//    }
//    std::cout <<endl;

    // if(tmp[1]<=-72){
    //     cout<<"轴2非安全点"<<endl;
    //     compute_robot_state.position[1]=(-72.0/180.0)*M_PI;
    // }
    // if(tmp[2]>=210){
    //     cout<<"轴3非安全点"<<endl;
    //     compute_robot_state.position[2]=(210.0/180.0)*M_PI;
    // }
    // if(tmp[4]<=-60){
    //     cout<<"轴5非安全点"<<endl;
    //     compute_robot_state.position[4]=(-60.0/180.0)*M_PI;
    // }
    compute_robot_state.header.stamp = ros::Time::now();

//    while(true)
//    {
        joint_state_pub->msg_ = compute_robot_state;
        joint_state_pub->unlockAndPublish();
//            break;
//        }
//    }

    

}

void forceService::getCurrentPose(geometry_msgs::Pose& Pose) {
    // 获取当前的末端姿态
    if(mode == ForceTechPoint) // 拖动示教 不断刷新点位
    {
        startPos = MG.move_group->getCurrentJointValues();
    }

    MG.kinematic_state->setJointGroupPositions( MG.joint_model_group, startPos);
    const Eigen::Affine3d &end_effector_state =  MG.kinematic_state->getGlobalLinkTransform( MG.endlinkName);
    tf::poseEigenToMsg(end_effector_state, Pose);
}

int forceService::computeImpedenceAndResult(std::vector<double> &force, std::vector<double> &outJoint, \
                                            geometry_msgs::Pose &outPose)
{
    assert(force.size() == 6);
    //1.获取当前位姿
    geometry_msgs::Pose current_Pose;
    getCurrentPose(current_Pose);

    //
    vector<double> Xa{0,0,0,0,0,0};
    computeImpedence(force, Xa);

    debugImpendXa(Xa);

    //3.位姿补偿计算
    geometry_msgs::Pose computePose = current_Pose;
    computePose.position.x+=Xa[0];
    computePose.position.y+=Xa[1];
    computePose.position.z+=Xa[2];

    //欧拉角偏执
    addImpendBiasZYX(computePose, Xa);

    outPose = std::move(computePose);
    return IKCompute(computePose, outJoint);
}

int forceService::computeAdjustImpedence(std::vector<double> &nextPose, std::vector<double> &force, std::vector<double> &outJoint, geometry_msgs::Pose &outPose)
{
    assert(force.size() == 6);
    //1.获取当前位姿
    geometry_msgs::Pose current_Pose;
    FKComputePose(nextPose, current_Pose);

//    cout<<"<--------------------------->"<<endl;
    cout<<"计算点位前 的关节给定量 : ";
    for(int i =0 ; i< 6 ;i++)
        std::cout << nextPose[i]*180/M_PI <<" ";
    std::cout <<std::endl;

    //2.
    vector<double> Xa{0,0,0,0,0,0};
    computeImpedence(force, Xa);

    debugImpendXa(Xa);

    //3.位姿补偿计算
    geometry_msgs::Pose computePose = current_Pose;
    computePose.position.x+=Xa[0];
    computePose.position.y+=Xa[1];
    computePose.position.z+=Xa[2];

    //欧拉角偏执
    addImpendBiasZYX(computePose, Xa);

    outPose = std::move(computePose);
    return IKCompute(outPose, outJoint);
}

void forceService::debugImpendXa(const std::vector<double> &XA)
{
//    cout<<"<--------------------------->"<<endl;
    cout<<"计算得偏移量_offset: ";
    for(int i =0 ; i< 6 ;i++)
        std::cout << XA[i] <<" ";
    std::cout <<std::endl;
}


int forceService::computeImpedence(std::vector<double> &force, std::vector<double> &outBiasJoint )
{

    //2.阻抗运算
    vector<double > tmp_pose{0,0,0,0,0,0}, Xa;
    force_plugin->setInputRobotPose(tmp_pose);
    force_plugin->setInputForceBias(force);

    cout<<"Force : ";
    for( int i = 0; i< 6; i++)
    {
        cout<<force[i]<<"  ";
    }
    cout<<endl;

    force_plugin->compute();
    force_plugin->getResult(Xa);
    //位移量控制
//    const double protectXValue = -0.04;
//    if(Xa[0]>=yamlPara_MaxVel_x){
//        Xa[0]=yamlPara_MaxVel_x;
//    }else if(Xa[0]<= protectXValue){
//        Xa[0]= protectXValue;
//    }

    outBiasJoint =  std::move(Xa);
    return 0;
}

void forceService::forceDataDeal(const vector<double >& original_force,vector<double >& deal_force) {
    assert(original_force.size() == 6);
    vector<double > tmp_deal_force(6);
    std::fill(tmp_deal_force.begin(),tmp_deal_force.begin()+6,0);

    if(!flag_SetForceBias){
        bias_force = currentForce;
        flag_SetForceBias = true;
    }

    //只管X,Y,Z三个方向力矩


    for (int i = 0; i < yamlPara_forceDrection.size(); ++i) {
        if(yamlPara_forceDrection[i]){
#ifdef REAL_FORCE
            tmp_deal_force[i] = original_force[i]-bias_force[i];
//            tmp_deal_force[i] = original_force[i] - 0; //测试代码
#else
            vector<double> fakeValue;
            fakeSensor->getForceVal(fakeValue);
            tmp_deal_force[i]  = fakeValue[i];
#endif
        }
    }
    deal_force= std::move(tmp_deal_force);
}

bool forceService::force_algorithmChangeCB(hirop_msgs::force_algorithmChange::Request &req,
                                           hirop_msgs::force_algorithmChange::Response &res) {
    yamlPara_algorithm_name=req.algorithm_name;
    res.is_success=true;
    return true;
}

void forceService::addImpendBiasZYX(geometry_msgs::Pose &computePose, const std::vector<double> &XA)
{
//    MotionCoorUlity::QtoE(computePose.orientation.x, computePose.orientation.y, computePose.orientation.z, computePose.orientation.w, A, B, C);
//    std::cout <<"compute offset pose: "<< X_offset<<" "<< Y_offset<<" "<< Z_offset<<" "<< A_offset<<" "<< B_offset<<" "<< C_offset<<" "<<std::endl;

//    std::cout << "计算前的 ABC "<< A<<" "<< B<<" "<< C<<" "<<std::endl;

     std::cout <<"计算前的 姿态: "<< computePose.orientation.w<<" "<< computePose.orientation.x<<" "\
              << computePose.orientation.y   <<" "<<computePose.orientation.z<<" "<<std::endl;


    // 偏移 calucation
//    A += XA[3]; B += XA[4]; B += XA[5];
//     偏移后RPY转四元数
    tf::Quaternion q, q0;
    tf::quaternionMsgToTF( computePose.orientation, q);

    double R, P, Y;
    tf::Matrix3x3(q).getEulerZYX(Y, P, R);
    std::cout << "计算前的 Z Y X "<< Y<<" "<< P<<" "<< R<<" "<<std::endl;

    //主要控制 Roll
#ifdef ZYX
    Y += XA[3]; P += XA[4]; R -= XA[5];
#else
    Y += XA[5]; P += XA[4]; R += XA[3];
#endif
    std::cout << "计算后的 Z Y X "<< Y<<" "<< P<<" "<< R<<" "<<std::endl;

    q0.setEulerZYX(Y, P, R);
//    tf::Vector3 axis = q0.getAxis();

    computePose.orientation.x = q0.getX();computePose.orientation.y = q0.getY();
    computePose.orientation.z = q0.getZ();computePose.orientation.w = q0.getW();
    std::cout <<"计算后的 姿态: "<< computePose.orientation.w<<" "<< computePose.orientation.x<<" "\
             << computePose.orientation.y   <<" "<<computePose.orientation.z<<" "<<std::endl;
}
