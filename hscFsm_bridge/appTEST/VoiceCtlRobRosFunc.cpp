#include "VoiceCtlRobRosFunc.h"

VoiceCtlRobRosFunc::VoiceCtlRobRosFunc(ros::NodeHandle *node):Node(node) {
    initRosToptic();
}

VoiceCtlRobRosFunc::~VoiceCtlRobRosFunc() {
}

void VoiceCtlRobRosFunc::initRosToptic(){
    //话题
    cout<<"----init topic----"<<endl;
    rosTopicHd.flag_forceSensor_publisher=Node->advertise<std_msgs::Bool>("forceSensor_moveFlag",1);
    rosTopicHd.shakehandOver_publisher=Node->advertise<std_msgs::Bool>("shake_over", 10);
    rosTopicHd.robStatusSend_publisher=Node->advertise<std_msgs::Bool>("uipub_robStatus",1);
    rosTopicHd.impedenceLive_publisher=Node->advertise<std_msgs::Bool>("uipub_impedenceLive",1);
    rosTopicHd.rbGoHome_publisher=Node->advertise<std_msgs::Int8>("homePoint",1);
    rosTopicHd.visionDetech_publisher=Node->advertise<std_msgs::Bool>("switch_of_vision_detect",1000);
    rosTopicHd.voice_order_publisher = Node->advertise<std_msgs::String>("voiceSolve_res", 1);
    rosTopicHd.robSpeedSet_publisher = Node->advertise<std_msgs::Float32>("speedScale", 1);

    rosTopicHd.voice_order_publisher = Node->advertise<std_msgs::String>("voiceSolve_res", 1);

    //服务
    rosTopicHd.RobReset_client = Node->serviceClient<hsr_rosi_device::ClearFaultSrv>("/clear_robot_fault");
    rosTopicHd.RobEnable_client = Node->serviceClient<hsr_rosi_device::SetEnableSrv>("/set_robot_enable");
    rosTopicHd.handClaw_gesture_client = Node->serviceClient<rb_msgAndSrv::rb_string>("handClaw_gesture");
    rosTopicHd.handClaw_shakeHand_client = Node->serviceClient<rb_msgAndSrv::rb_DoubleBool>("handClaw_shakeHand");
    rosTopicHd.handClaw_grabDoll_client = Node->serviceClient<rb_msgAndSrv::rb_DoubleBool>("handClaw_grabDoll");
    rosTopicHd.rob_goHome_client = Node->serviceClient<rb_msgAndSrv::rb_DoubleBool>("rob_goHome");
    rosTopicHd.RobSetMode_client = Node->serviceClient<hsr_rosi_device::setModeSrv>("/set_mode_srv");
    rosTopicHd.robGetStatus_client = Node->serviceClient<hirop_msgs::robotError>("getRobotErrorFaultMsg");
    rosTopicHd.personDetect_client = Node->serviceClient<rb_msgAndSrv::rb_EmptyAndInt>("get_people_track_state");

    //开关语音检测与图像检测
    rosTopicHd.switch_personDetect_client=Node->serviceClient<rb_msgAndSrv::rb_DoubleBool>("switch_personDetect");
    rosTopicHd.switch_voiceDetect_client=Node->serviceClient<rb_msgAndSrv::rb_DoubleBool>("switch_voiceDetect");

    rosTopicHd.backHomeClient = Node->serviceClient<std_srvs::SetBool>("/back_home");
    rosTopicHd.detectePointClient = Node->serviceClient<rb_msgAndSrv::rb_DoubleBool>("/handClaw_detectDoll");
    rosTopicHd.stopMotionClient = Node->serviceClient<industrial_msgs::StopMotion>("/stop_motion");
    //抓娃娃
    rosTopicHd.detectionClient = Node->serviceClient<hirop_msgs::detection>("/detection");
    // 订阅
    //rosTopicHd.pickServer_client = Node->serviceClient<pick_place_bridge::PickPlacePose>("pick");
    //rosTopicHd.placeServer_client = Node->serviceClient<pick_place_bridge::PickPlacePose>("place");
    rosTopicHd.sayGoodByeAction_client = Node->serviceClient<std_srvs::SetBool>("sayGoodByeAction");
    rosTopicHd.robOKAction_client = Node->serviceClient<std_srvs::SetBool>("/ok_pose");
    rosTopicHd.setByeByeAction_client = Node->serviceClient<hirop_msgs::setStartUpProject>("/setStartUpProject");
    rosTopicHd.setByeByeInterrupt_client = Node->serviceClient<hirop_msgs::setStopProject>("/setStopProject");

    rosTopicHd.StartImpedence_client=Node->serviceClient<std_srvs::SetBool>("force_bridge/impedenceStart");
    rosTopicHd.CloseImpedence_client=Node->serviceClient<std_srvs::SetBool>("force_bridge/impedenceClose");
    rosTopicHd.startShakeHandJudge_client=Node->serviceClient<hirop_msgs::shakeHandSet>("shakeHandJudge/begin");
    rosTopicHd.closeShakeHandJudge_client=Node->serviceClient<std_srvs::SetBool>("shakeHandJudge/end");

    rosTopicHd.jointMultiClient = Node->serviceClient<hirop_msgs::moveToMultiPose>("motion_bridge/moveToMultiPose");
    rosTopicHd.exeTrajectoryClinet = Node->serviceClient<hirop_msgs::dualRbtraject>("motion_bridge/sigRobMotion_JointTraject");                                                                               
    rosTopicHd.loadPoseDataClient = Node->serviceClient<hirop_msgs::loadPoseData>("/load_pose_data");
    rosTopicHd.loadJointDataClient = Node->serviceClient<hirop_msgs::loadJointsData>("/load_joint_data");
    rosTopicHd.jointPlannerClinet = Node->serviceClient<hirop_msgs::addJointPose>("/trajectory_planner/jointSpacePlanner");
    rosTopicHd.cartesianPlannerClient = Node->serviceClient<hirop_msgs::addPose>("/trajectory_planner/cartesianPlanner");
    rosTopicHd.getTrajectoryClinet = Node->serviceClient<hirop_msgs::getTrajectory>("/trajectory_planner/getPlannTrajectory");

    Node->param("/handgesture", handgestureFile, std::string("handgesturePose"));
    Node->param("/home", homeFile, std::string("homePose"));

    loadRobotPose(handgesturePose, handgestureFile);
    loadRobotPose(HomeJointsPose, homeFile);

    // 接受者
    rosTopicHd.rbCtlBusy_subscriber=Node->subscribe<std_msgs::Bool>("rbCtlBusy_status",1,&VoiceCtlRobRosFunc::callback_rbCtlBusy_status_subscriber, this);
    rosTopicHd.robStatus_subscriber=Node->subscribe<industrial_msgs::RobotStatus>("robot_status",1,boost::bind(&VoiceCtlRobRosFunc::callback_robStatus_subscriber,this,_1));
    rosTopicHd.isOpenFollow_subscriber=Node->subscribe<std_msgs::Bool>("is_follow",1,&VoiceCtlRobRosFunc::callback_isOpenFollow_subscriber,this);
    rosTopicHd.getShakeHandResult_subscriber = Node->subscribe<std_msgs::Int16>("impedance_result",1,&VoiceCtlRobRosFunc::callback_getShakeResult_subscriber,this);
    rosTopicHd.objectArraySub = Node->subscribe<hirop_msgs::ObjectArray>("object_array", 1, &VoiceCtlRobRosFunc::callback_objectCallBack, this);
    rosTopicHd.voice_order_sub = Node->subscribe("/voice_order", 1, &VoiceCtlRobRosFunc::voice_order_CallBack, this);
    rosTopicHd.people_detect_sub = Node->subscribe("/pedestrian_detection", 1, &VoiceCtlRobRosFunc::people_detect_CallBack, this);
    rosTopicHd.shakehandstatus_sub = Node->subscribe("shakeHandJudge/Status", 1, &VoiceCtlRobRosFunc::shakehandstatus_CallBack, this);


}

void VoiceCtlRobRosFunc::callback_robStatus_subscriber(const industrial_msgs::RobotStatus::ConstPtr robot_status){
    if(robot_status->in_error.val==0){
        statemonitor.RobNormalState=true;
    } else
    {
        statemonitor.RobNormalState= false;
    }
    if(robot_status->drives_powered.val==1){
        statemonitor.RobEnableState= true;
    } else{
        statemonitor.RobEnableState= false;
    }
}

void VoiceCtlRobRosFunc::callback_rbCtlBusy_status_subscriber(std_msgs::Bool msg){
    //机器人控制模块运动中
    if(msg.data){
        statemonitor.flag_rbCtlBusy= true;
    }
    else{
        statemonitor.flag_rbCtlBusy= false;
    }
}
void VoiceCtlRobRosFunc::callback_isOpenFollow_subscriber(std_msgs::Bool msg){
    statemonitor.isOk_robPreparePose=msg.data;
}
void VoiceCtlRobRosFunc::callback_getShakeResult_subscriber(std_msgs::Int16 msg) {
    std::cout << "get ShakeReult :" << msg.data << std::endl;
    if (msg.data == 0) {
        statemonitor.isEnd_shakeHand = true;
    }
}

void VoiceCtlRobRosFunc::callback_objectCallBack(hirop_msgs::ObjectArray obj){
    hirop_msgs::ObjectInfo pose = obj.objects.at(0);
    geometry_msgs::PoseStamped pp2 = pose.pose;
    transformFrame(pp2, "world");
    statemonitor.object_pose = pp2;
    std::cout << "<-- objectCallBack ShakeReult ---"<<std::endl;
}

/******************************话题回调函数**************************************************/

void VoiceCtlRobRosFunc::voice_order_CallBack(const std_msgs::Int16::ConstPtr & msg)
{
    statemonitor.voice_order = msg->data;
}

void VoiceCtlRobRosFunc::people_detect_CallBack(const std_msgs::Bool::ConstPtr & msg)
{
    if(msg->data)
    {
        statemonitor.hasPeople = 1;
    }else
    {
        statemonitor.hasPeople = 0;
    }
}

void VoiceCtlRobRosFunc::shakehandstatus_CallBack(const hirop_msgs::shakeHandStatus::ConstPtr msg){
    statemonitor.isEnd_shakeHand=msg->shakeHand_over;
}




//功能函数
bool VoiceCtlRobRosFunc::transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id)
{

    geometry_msgs::PoseStamped* worldFramePose = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped* otherFramePose = new geometry_msgs::PoseStamped[1];
    tf::TransformListener listener;

    otherFramePose[0] = poseStamped;
    for(int i=0; i < 5; ++i)
    {
        try
        {
            listener.transformPose(frame_id, otherFramePose[0], worldFramePose[0]);
            break;
        }
        catch(tf::TransformException& ex)
        {
            ROS_INFO_STREAM(ex.what());
            ros::WallDuration(1).sleep();
            continue;
        }
    }
    poseStamped = worldFramePose[0];
    poseStamped.pose.orientation.x = 0;
    poseStamped.pose.orientation.y = 0;
    poseStamped.pose.orientation.z = 0;
    poseStamped.pose.orientation.w = 1;
    delete[] worldFramePose;
    delete[] otherFramePose;
    double add[3] = {0};
    Node->getParam("/grasp_place/position_x_add", add[0]);
    Node->getParam("/grasp_place/position_y_add", add[1]);
    Node->getParam("/grasp_place/position_z_add", add[2]);

    poseStamped.pose.position.x += add[0];
    poseStamped.pose.position.y += add[1];
    poseStamped.pose.position.z += add[2];
    if(poseStamped.header.frame_id == "world")
    {
        return true;
    }
    else
    {
        return false;
    }
}
int VoiceCtlRobRosFunc::robGotoShakeHandPose(){
    hsr_rosi_device::setModeSrv srv_SetMode;
    srv_SetMode.request.mode = 0;
    if (rosTopicHd.RobSetMode_client.call(srv_SetMode))
    {
        if (!srv_SetMode.response.finsh)
        {
            cout << "随动模式设置失败" << endl;
            return -1;
        }
    }
    else
    {
        cout<<"模式设置服务连接失败"<<endl;
        return -1;
    }

    int flag = -1;
    if(!handgesturePose.empty())
    {
        flag = movePose(handgesturePose);
    }
    return flag;
}

int VoiceCtlRobRosFunc::startImpedence(){
    cout<<"开启阻抗"<<endl;
    //设置随动模式
    hsr_rosi_device::setModeSrv srv_SetMode;
    srv_SetMode.request.mode = 1;
    if (rosTopicHd.RobSetMode_client.call(srv_SetMode))
    {
        if (!srv_SetMode.response.finsh)
        {
            cout << "随动模式设置失败" << endl;
            return -1;
        }
    }
    else
    {
        cout<<"模式设置服务连接失败"<<endl;
        return -1;
    }
    hirop_msgs::shakeHandSet srv_shakeset;
    srv_shakeset.request.MaxDistance=0.05;
    srv_shakeset.request.MinDistance=0.01;
    srv_shakeset.request.countTime=2;
    rosTopicHd.startShakeHandJudge_client.call(srv_shakeset);
    //打开阻抗
    std_srvs::SetBool srv_startImp;
    srv_startImp.request.data=true;
    //
    if(rosTopicHd.StartImpedence_client.call(srv_startImp)){
        if(!srv_startImp.response.success){
            ROS_INFO("open impedence is_success failed ");
            return  -1;
        }
    }else{
        ROS_INFO("open impedence failed ");
    }
    cout<<"阻抗开启结束"<<endl;
    return 0;
}

int VoiceCtlRobRosFunc::closeImpedence() {
    //关闭阻抗
    cout<<"关闭阻抗"<<endl;
    std_srvs::SetBool srv_end;
    srv_end.request.data=true;
    rosTopicHd.closeShakeHandJudge_client.call(srv_end);
    std_srvs::SetBool srv_CloseImp;
    srv_CloseImp.request.data=true;
    if(rosTopicHd.CloseImpedence_client.call(srv_CloseImp)){
        if(!srv_CloseImp.response.success){
            cout<<"CloseImpedence_client关闭阻抗失败"<<endl;
            return -1;
        }
    } else{
        cout<<"CloseImpedence_client服务连接失败"<<endl;
        return -1;
    }

    hsr_rosi_device::setModeSrv srv_SetMode;
    srv_SetMode.request.mode=0;
    if(rosTopicHd.RobSetMode_client.call(srv_SetMode)){
        cout<<"设置为点动模式"<<endl;
    } else
    {
        cout<<"模式设置服务连接失败"<<endl;
        return -1;
    }
    system("rosservice call /stop_motion ");
    return 0;
}

int VoiceCtlRobRosFunc::robGotoPhotoPose(){
    if(RobGoHome()){
        cout<<"回原点成功"<<endl;
    } else
    {
        cout<<"回原点失败"<<endl;
        return -1 ;
    }

    rb_msgAndSrv::rb_DoubleBool  dSrv;
    dSrv.request.request = true;
    if(!rosTopicHd.detectePointClient.call(dSrv))
    {
        ROS_INFO_STREAM("check handClaw_detectDoll server");
        return -1;
    }
    else
    {
        if(dSrv.response.respond)
        {
            ROS_INFO_STREAM("move to detete point SUCCESS");
        }
        else
        {
            ROS_INFO("move to detect point FAILURE");
            return -1;
        }
    }
    return 0;
}

int VoiceCtlRobRosFunc::detectToy(){
    hirop_msgs::detection d;
    d.request.detectorName = "Yolo6d";
    d.request.detectorType = 1;
    //d.request.objectName = "bluerabbit";
    d.request.objectName = "toy1";

    if(rosTopicHd.detectionClient.call(d))
    {
        if(d.response.result==0){
            cout<<"识别成功"<<endl;
        }else
        {
            cout<<"识别失败"<<endl;
            return -1;
        }
    }else
    {
        cout<<"detectionClient服务连接失败"<<endl;
        return -1;
    }

    return 0;
}

int VoiceCtlRobRosFunc::robGrabToy() {
    // /*************************************************************************/
    // pick_place_bridge::PickPlacePose srv;
    // srv.request.Pose = statemonitor.object_pose;
    // std::cout << "<- objectCallBack ShakeReult Z "<<srv.request.Pose.pose.position.z<<std::endl;

    // if(srv.request.Pose.pose.position.z < 1.1){
    //     cout<<"轨迹规划点位过低,请检测";
    //     return -1;
    // }
    // rosTopicHd.pickServer_client.call(srv);
    // if(srv.response.result != true){
    //     std::cout << "planning pick error  ---"<<std::endl;
    //     return -1;
    // }
    // ROS_INFO_STREAM("****************************************************");
    // /*******************************/
    // srv.request.Pose.pose.position.x = 0.95;
    // srv.request.Pose.pose.position.y = -0.45;
    // srv.request.Pose.pose.position.z = 1.51;
    // srv.request.Pose.pose.orientation.x = 0;
    // srv.request.Pose.pose.orientation.y = 0;
    // srv.request.Pose.pose.orientation.z = 0;
    // srv.request.Pose.pose.orientation.w = 1;
    // rosTopicHd.placeServer_client.call(srv);
    // if(srv.response.result != true){
    //     std::cout << "place pick error  ---"<<std::endl;
    //     return -1;
    // }
    return 0;
}

int VoiceCtlRobRosFunc::RobGoHome(){
    int flag = -1;
    if(!HomeJointsPose.empty())
    {
        flag = movePose(HomeJointsPose);
    }
    return flag;
}

void VoiceCtlRobRosFunc::RobSayWords(std::string words){
    std_msgs::String se_msg;
    se_msg.data = words.c_str();
    rosTopicHd.voice_order_publisher.publish(se_msg);
}

StateMonitor VoiceCtlRobRosFunc::getStateMonitor() {

    return statemonitor;
}

void VoiceCtlRobRosFunc::initStateMonitor(){
    statemonitor.RobNormalState= false;
    statemonitor.voice_order= -1;
    statemonitor.flag_rbCtlBusy= false;
    statemonitor.isEnd_shakeHand= false;
    statemonitor.isOk_robPreparePose= false;
    statemonitor.RobEnableState= false;
    statemonitor.grab_ok= false;
    statemonitor.hasPeople= 0;
    geometry_msgs::PoseStamped stamped;
    statemonitor.object_pose=stamped;
}

int VoiceCtlRobRosFunc::startShakeHandJudge() {
    hirop_msgs::shakeHandSet srv;
    srv.request.MaxDistance=0.04;
    srv.request.MinDistance=0.005;
    srv.request.countTime=2;
    rosTopicHd.startShakeHandJudge_client.call(srv);
    return 0;
}

int VoiceCtlRobRosFunc::closeShakeHandJudge() {
    std_srvs::SetBool srv;
    srv.request.data=true;
    rosTopicHd.closeShakeHandJudge_client.call(srv);
    return 0;
}

void VoiceCtlRobRosFunc::PersonDetect_Switch(bool flag) {
    if(flag){
        system("rosrun openni2_tracker peopledetection.sh &");
    } else{
        std_msgs::Bool msg;
        msg.data= false;
        rosTopicHd.visionDetech_publisher.publish(msg);
    }
}

void VoiceCtlRobRosFunc::VoiceDetect_Switch(bool flag) {
    rb_msgAndSrv::rb_DoubleBool srv_witchVoiceDetect;
    srv_witchVoiceDetect.request.request= flag;
    rosTopicHd.switch_voiceDetect_client.call(srv_witchVoiceDetect);
}

int VoiceCtlRobRosFunc::loadRobotPose(std::vector<geometry_msgs::PoseStamped>& poses, std::string fileName)
{
    hirop_msgs::loadPoseData srv;
    srv.request.uri = "five_finger";
    srv.request.name = fileName;
    int flag = 0;
    if(rosTopicHd.loadPoseDataClient.call(srv))
    {
        ROS_INFO_STREAM("load " << fileName << " success");
        poses = srv.response.poses;
    }
    else
    {
        ROS_INFO_STREAM("load " << fileName << " failed");
        flag = -1;
    }
    return flag;
}

int VoiceCtlRobRosFunc::loadRobotPose(std::vector<std::vector<double> >& joints,std::string fileName)
{
    hirop_msgs::loadJointsData srv;
    srv.request.uri = "five_finger";
    srv.request.name = fileName;
    int flag = 0;
    if(rosTopicHd.loadJointDataClient.call(srv))
    {
        ROS_INFO_STREAM("load " << fileName << " success");
        int size = srv.response.joints.size();
        joints.resize(size);
        int oneSize = srv.response.joints[0].joint.size();
        for(int i=0; i<size; i++)
        {
            joints[i].resize(oneSize);
            for(int j=0; j<oneSize; j++)
            {
                joints[i][j] = (srv.response.joints[i].joint[j]/M_PI)*180;
            }
        }
    }
    else
    {
        ROS_INFO_STREAM("load " << fileName << " failed");
        flag = -1;
    }
    return flag;
}

int VoiceCtlRobRosFunc::movePose(std::vector<geometry_msgs::PoseStamped>& poses)
{
    int flag = -1;
    if(cartisianPlanner(poses) == 0)
    {
        moveit_msgs::RobotTrajectory tra;
        if(getTrajectory(tra) == 0)
        {
            flag = motionMulti(tra);
        }
    }
    return flag;
}

int VoiceCtlRobRosFunc::movePose(std::vector<std::vector<double> >& joints)
{
    int flag = -1;
    if(jointSpacePlanner(joints) == 0)
    {
        moveit_msgs::RobotTrajectory tra;
        if(getTrajectory(tra) == 0)
        {
            flag = motionMulti(tra);

        }
    }
    return flag;
}

int VoiceCtlRobRosFunc::cartisianPlanner(std::vector<geometry_msgs::PoseStamped>& poses)
{
    int flag = 0;
    hirop_msgs::addPose cartisianSrv;
    cartisianSrv.request.pose = poses;
    cartisianSrv.request.type.push_back(0);
    if(rosTopicHd.cartesianPlannerClient.call(cartisianSrv))
    {
        if(cartisianSrv.response.result != 0)
        {
            ROS_INFO("planning failed");
            flag =  -1;
        }
    }
    else
    {
        ROS_INFO("check planner service");
        flag =  -1;
    }
    return flag;
}

int VoiceCtlRobRosFunc::jointSpacePlanner(std::vector<std::vector<double> >& joints)
{
    int flag = 0;
    hirop_msgs::addJointPose jointsSpaceSrv;
    jointsSpaceSrv.request.joints.resize(joints.size());
    for(int i=0; i<joints.size(); i++)
    {
        jointsSpaceSrv.request.joints[i].joint = joints[i];

    }
    if(rosTopicHd.jointPlannerClinet.call(jointsSpaceSrv))
    {
        if(jointsSpaceSrv.response.result != 0)
        {
            ROS_INFO("planning failed");
            flag =  -1;
        }
    }
    else
    {
        ROS_INFO("check planner service");
        flag =  -1;
    }
    return flag;
}

int VoiceCtlRobRosFunc::getTrajectory(moveit_msgs::RobotTrajectory& tra)
{
    int flag = 0;
    hirop_msgs::getTrajectory getTraSrv;
    if(!rosTopicHd.getTrajectoryClinet.call(getTraSrv))
    {
        ROS_INFO("get trajectory failed");
        flag = -1;
    }
    else
    {
        tra.joint_trajectory.header = getTraSrv.response.tarjectory.joint_trajectory.header;
        tra.joint_trajectory.joint_names = getTraSrv.response.tarjectory.joint_trajectory.joint_names;
        for(int i= 0; i<getTraSrv.response.tarjectory.joint_trajectory.points.size(); i++)
        {
            tra.joint_trajectory.points.push_back(getTraSrv.response.tarjectory.joint_trajectory.points[i]);
        }
    }
    return flag;
}

int VoiceCtlRobRosFunc::motionMulti(moveit_msgs::RobotTrajectory& tra)
{
    int flag = 0;
//    hirop_msgs::moveToMultiPose moveSrv;
//    moveSrv.request.moveGroup_name = "arm";
//    int size = tra.joint_trajectory.points.size();
//    moveSrv.request.poseList_joints_angle.resize(size);
//    for(int i=0; i<size; i++)
//    {
//        moveSrv.request.poseList_joints_angle[i].joints_angle.data = tra.joint_trajectory.points[i].positions;
//    }
//    if(!rosTopicHd.jointMultiClient.call(moveSrv))
//    {
//        ROS_INFO_STREAM("check move pose server");
//        flag = -1;
//    }
//    else
//    {
//        if(moveSrv.response.is_success)
//        {
//            ROS_INFO_STREAM("move pose SUCCESS");
//        }
//        else
//        {
//            ROS_INFO_STREAM("move pose FAILED");
//            flag = -1;
//        }
//    }
     hirop_msgs::dualRbtraject dTra;
     dTra.request.robotMotionTraject_list.resize(1);
     dTra.request.robotMotionTraject_list[0].moveGroup_name = "arm";
     dTra.request.robotMotionTraject_list[0].robot_jointTra = tra.joint_trajectory;
     if(!rosTopicHd.exeTrajectoryClinet.call(dTra))
     {
         ROS_INFO_STREAM("check move pose server");
         flag = -1;
     }
     else
     {
         if(dTra.response.is_success)
         {
             ROS_INFO_STREAM("move pose SUCCESS");
         }
         else
         {
             ROS_INFO_STREAM("move pose FAILED");
             flag = -1;
         }
     }
    return flag;
}


