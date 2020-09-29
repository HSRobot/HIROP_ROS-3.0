#include <VisualCaptureRosFunc.h>

using namespace std;

VisualCaptureRosFunc::VisualCaptureRosFunc(ros::NodeHandle *node) : Node(node)
{
    RosInit();
    StateMonitorInit();
}

VisualCaptureRosFunc::~VisualCaptureRosFunc()
{
    system("rostopic pub -1 /switch_of_people_detect std_msgs/Bool \"data: false\"");
}

void VisualCaptureRosFunc::RosInit()
{
    RobotResetClient = Node->serviceClient<hsr_rosi_device::ClearFaultSrv>("/clear_robot_fault");
    RobotEnableClient = Node->serviceClient<hsr_rosi_device::SetEnableSrv>("/set_robot_enable");
    detectClient = Node->serviceClient<hirop_msgs::detection>("/detection");
    pickClient = Node->serviceClient<hirop_msgs::Pick>("/pickplace_bridge/pick");
    placeClient = Node->serviceClient<hirop_msgs::Place>("/pickplace_bridge/place");
    loadPoseDataClient = Node->serviceClient<hirop_msgs::loadPoseData>("/load_pose_data");
    loadJointDataClient = Node->serviceClient<hirop_msgs::loadJointsData>("/load_joint_data");
    jointPlannerClient = Node->serviceClient<hirop_msgs::addJointPose>("/trajectory_planner/jointSpacePlanner");
    CartesianPlannerClient = Node->serviceClient<hirop_msgs::addPose>("/trajectory_planner/cartesianPlanner");
    getTrajectoryClinet = Node->serviceClient<hirop_msgs::getTrajectory>("/trajectory_planner/getPlannTrajectory");
    jointMultiClient = Node->serviceClient<hirop_msgs::moveToMultiPose>("/motion_bridge/moveToMultiPose");
    exeTrajectoryClient = Node->serviceClient<hirop_msgs::dualRbtraject>("/motion_bridge/sigRobMotion_JointTraject");
    stopMotionClient = Node->serviceClient<industrial_msgs::StopMotion>("/stop_motion");
    lookPointCloudClient = Node->serviceClient<hirop_msgs::LookPointCloud>("/lookPointCloud");

    setVAClient = Node->serviceClient<hirop_msgs::setVelocityAccelerated>("/pickplace_bridge/setVelocityAccelerated");
    setParamClient = Node->serviceClient<hirop_msgs::updateParam>("/trajectory_planner/updatePlannerParam");

    shutDownPedsDetectionPub = Node->advertise<std_msgs::Bool>("/switch_of_people_detect", 10);

    RobotStatusSub = Node->subscribe<industrial_msgs::RobotStatus>("/robot_status", 1, boost::bind(&VisualCaptureRosFunc::robotStatusSub_CallBack, this, _1));
    ObjectArraySub = Node->subscribe<hirop_msgs::ObjectArray>("/object_array", 10, &VisualCaptureRosFunc::objectDetectSub_CallBack, this);
    detectHumanSub = Node->subscribe<std_msgs::Bool>("/people_detection_feedback", 10, &VisualCaptureRosFunc::detectHuman_Callback, this);

    Node->param("/home", HomeFile, string("homePose"));
    Node->param("/detection", DetectionFile, string("detectionPose2"));
    Node->param("/place", PlaceObjFile, string("PlaceObjPose"));

    VisualCaptureRosFunc::loadRobotPose(HomePoses, HomeFile);
    VisualCaptureRosFunc::loadRobotPose(DetectionPoses, DetectionFile);
    VisualCaptureRosFunc::loadRobotPose(PlaceObjPoses, PlaceObjFile);
    isPedsdetetion = false;
}

void VisualCaptureRosFunc::StateMonitorInit()
{
    statemonitor.RobotStatus = false;
    statemonitor.RobotEnableStatus = false;
    statemonitor.havePickPose = false;
}

VC_StateMonitor VisualCaptureRosFunc::getStateMonitor()
{
    return statemonitor;
}

void VisualCaptureRosFunc::robotStatusSub_CallBack(const industrial_msgs::RobotStatus::ConstPtr robot_statu)
{
    if (robot_statu->in_error.val == 0)
    {
        statemonitor.RobotStatus = true;
    }
    else
    {
        statemonitor.RobotStatus = false;
    }

    if (robot_statu->drives_powered.val == 1)
    {
        statemonitor.RobotEnableStatus = true;
    }
    else
    {
        statemonitor.RobotEnableStatus = false;
    }

    //    cout << "Current RobotStatus: " << statemonitor.RobotStatus << endl;
    //    cout << "Current RobotEnableStatus: " << statemonitor.RobotEnableStatus << endl;
}

int VisualCaptureRosFunc::detect(string &object_name)
{
    cout << "开始检测" << endl;
    int detect_result = 0;

    hirop_msgs::detection detect_srv;
    detect_srv.request.objectName = object_name;
    detect_srv.request.detectorName = "Yolo6d";
    detect_srv.request.detectorType = 1;
    detect_srv.request.detectorConfig = "";

    if (detectClient.call(detect_srv))
    {
        if (detect_srv.response.result != 0)
        {
            detect_result = -1;
            cout << "识别错误" << endl;
        }
    }
    else
    {
        detect_result = -1;
        cout << "请检查有无此/detection服务" << endl;
    }

    cout << "检测完毕" << endl;
    return detect_result;
}

void VisualCaptureRosFunc::objectDetectSub_CallBack(const hirop_msgs::ObjectArrayConstPtr &msg)
{
    //先清空上一次的抓取点缓存
    vector<geometry_msgs::PoseStamped>().swap(PickObjPoses);

    for (int i = 0; i < msg->objects.size(); ++i)
    {
        PickObjPoses.push_back(msg->objects[i].pose);
        VisualCaptureRosFunc::transformFrame(PickObjPoses[i], "world");
    }

    statemonitor.havePickPose = true;
    cout << "抓取目标点信息生成成功！" << endl;
}

void VisualCaptureRosFunc::detectHuman_Callback(const std_msgs::Bool::ConstPtr &msg)
{
    static bool before = false, now = true;
    now = msg->data;
    // ROS_ERROR("into callback");
    if (before != now)
    {
        hirop_msgs::setVelocityAccelerated VASrv;
        hirop_msgs::updateParam paramSrv;
        if (msg->data)
        {
            ROS_ERROR("Velocity: 0.1");
            VASrv.request.Velocity = 0.1;
            VASrv.request.Accelerated = 0.1;
            paramSrv.request.file = "config";
        }
        else
        {
            ROS_ERROR("Velocity: 1");
            VASrv.request.Accelerated = 1;
            VASrv.request.Velocity = 1;
            paramSrv.request.file = "config2";
        }
        if (!setVAClient.call(VASrv))
        {
            ROS_INFO("check pickplace_bridge node");
        }
        if (!setParamClient.call(paramSrv))
        {
            ROS_INFO("check planning_bridge node");
        }
        before = now;
    }
}

bool VisualCaptureRosFunc::transformFrame(geometry_msgs::PoseStamped &p, string frame_id)
{
    geometry_msgs::PoseStamped *target_pose = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped *source_pose = new geometry_msgs::PoseStamped[1];
    tf::TransformListener tf_listener;

    source_pose[0] = p;
    for (int i = 0; i < 5; ++i)
    {
        try
        {
            tf_listener.transformPose(frame_id, source_pose[0], target_pose[0]);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("transfrom exception : %s", ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
    }

    p = target_pose[0];
    p.pose.position.z = 1.60;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;

    delete[] target_pose;
    delete[] source_pose;

    if (p.header.frame_id == "world")
    {
        return true;
    }
    else
    {
        return false;
    }
}

void VisualCaptureRosFunc::RobotReset()
{
    hsr_rosi_device::ClearFaultSrv robotReset_srv;
    if (!RobotResetClient.call(robotReset_srv))
    {
        ROS_ERROR("check ClearFault service");
        return;
    }
}

void VisualCaptureRosFunc::RobotEnable(bool value)
{
    hsr_rosi_device::SetEnableSrv robotEnable_srv;
    robotEnable_srv.request.enable = value;
    if (!RobotEnableClient.call(robotEnable_srv))
    {
        ROS_ERROR("check SetEnabl service");
        return;
    }
}

void VisualCaptureRosFunc::RobotStopMotion()
{
    industrial_msgs::StopMotion stopMotion_srv;
    if (!stopMotionClient.call(stopMotion_srv))
    {
        ROS_ERROR("check /stop_motion service service");
        return;
    }
}

int VisualCaptureRosFunc::RobotGoHome()
{
    int result = -1;
    if (!HomePoses.empty())
    {
        for (int i = 0; i < 3; ++i)
        {
            result = VisualCaptureRosFunc::movePose(HomePoses);
            if (result == 0)
                break;
        }
    }
    else
    {
        ROS_ERROR("HomePoses加载数据为空");
    }

    return result;
}

int VisualCaptureRosFunc::RobotGoDetection()
{
    int result = -1;

    if (!DetectionPoses.empty())
    {
        result = VisualCaptureRosFunc::movePose(DetectionPoses);
    }
    else
    {
        ROS_ERROR("DetectionPoses加载数据为空");
    }

    return result;
}

int VisualCaptureRosFunc::RobotPick(geometry_msgs::PoseStamped &pose)
{
    int result = 0;

    hirop_msgs::Pick pick_srv;
    pick_srv.request.pickPos = pose;
    if (pickClient.call(pick_srv))
    {
        if (!pick_srv.response.isPickFinsh)
        {
            cout << "无法规划到抓取点" << endl;
            result = -1;
        }
    }
    else
    {
        ROS_ERROR("check Pick service!");
        result = -1;
    }

    return result;
}

int VisualCaptureRosFunc::RobotPlace(geometry_msgs::PoseStamped &pose)
{
    int result = 0;

    hirop_msgs::Place place_srv;
    place_srv.request.placePos = pose;
    if (placeClient.call(place_srv))
    {
        if (!place_srv.response.isPlaceFinsh)
        {
            result = -1;
        }
    }
    else
    {
        ROS_ERROR("check Place service!");
        result = -1;
    }

    return result;
}

/***机器人运动控制函数***/

int VisualCaptureRosFunc::loadRobotPose(vector<geometry_msgs::PoseStamped> &poses, string fileName)
{
    hirop_msgs::loadPoseData loadpose_srv;
    loadpose_srv.request.uri = "five_finger";
    loadpose_srv.request.name = fileName;

    int result = 0;

    if (loadPoseDataClient.call(loadpose_srv))
    {
        ROS_INFO_STREAM("load " << fileName << " successfully!");
        poses = loadpose_srv.response.poses;
    }
    else
    {
        ROS_INFO_STREAM("load " << fileName << " failed");
        result = -1;
    }

    return result;
}

int VisualCaptureRosFunc::loadRobotPose(vector<vector<double>> &joints, string fileName)
{
    hirop_msgs::loadJointsData loadjoint_srv;
    loadjoint_srv.request.uri = "five_finger";
    loadjoint_srv.request.name = fileName;
    int result = 0;

    if (loadJointDataClient.call(loadjoint_srv))
    {
        ROS_INFO_STREAM("load " << fileName << " successfully!");

        int size = loadjoint_srv.response.joints.size();
        joints.resize(size);
        int oneSize = loadjoint_srv.response.joints[0].joint.size();
        for (int i = 0; i < size; ++i)
        {
            joints[i].resize(oneSize);
            for (int j = 0; j < oneSize; ++j)
            {
                joints[i][j] = (loadjoint_srv.response.joints[i].joint[j] / M_PI) * 180;
            }
        }
    }
    else
    {
        ROS_INFO_STREAM("load " << fileName << " failed");
        int result = -1;
    }

    return result;
}

int VisualCaptureRosFunc::movePose(vector<geometry_msgs::PoseStamped> &poses)
{
    int result = -1;

    if (CartesianPlanner(poses) == 0)
    {
        moveit_msgs::RobotTrajectory tra;
        if (VisualCaptureRosFunc::getTrajectory(tra) == 0)
        {
            result = VisualCaptureRosFunc::motionMulti(tra);
        }
    }

    return result;
}

int VisualCaptureRosFunc::movePose(vector<vector<double>> &joints)
{

    int result = -1;
    if (VisualCaptureRosFunc::JointSpacePlanner(joints) == 0)
    {
        moveit_msgs::RobotTrajectory tra;
        if (VisualCaptureRosFunc::getTrajectory(tra) == 0)
        {
            result = VisualCaptureRosFunc::motionMulti(tra);
        }
    }

    return result;
}

int VisualCaptureRosFunc::CartesianPlanner(vector<geometry_msgs::PoseStamped> &poses)
{
    int result = 0;

    hirop_msgs::addPose cartesian_srv;
    cartesian_srv.request.pose = poses;
    cartesian_srv.request.type.push_back(0);
    if (CartesianPlannerClient.call(cartesian_srv))
    {
        if (cartesian_srv.response.result != 0)
        {
            ROS_ERROR("CartesianSpace planning failed!");
            result = -1;
        }
    }
    else
    {
        ROS_ERROR("check CartesianPlanner service!");
        result = -1;
    }

    return result;
}

int VisualCaptureRosFunc::JointSpacePlanner(vector<vector<double>> &joints)
{
    int result = 0;

    hirop_msgs::addJointPose jointSpace_srv;
    jointSpace_srv.request.joints.resize(joints.size());
    for (int i = 0; i < joints.size(); ++i)
    {
        jointSpace_srv.request.joints[i].joint = joints[i];
    }

    if (jointPlannerClient.call(jointSpace_srv))
    {
        if (jointSpace_srv.response.result != 0)
        {
            ROS_ERROR("JointSpace planning failed!");
            result = -1;
        }
    }
    else
    {
        ROS_ERROR("check JointPlanner service!");
        result = -1;
    }

    return result;
}

int VisualCaptureRosFunc::getTrajectory(moveit_msgs::RobotTrajectory &tra)
{
    int result = 0;

    hirop_msgs::getTrajectory getTra_srv;
    if (!getTrajectoryClinet.call(getTra_srv))
    {
        ROS_ERROR("check getTrajectory service!");
        result = -1;
    }
    else
    {
        tra.joint_trajectory.header = getTra_srv.response.tarjectory.joint_trajectory.header;
        tra.joint_trajectory.joint_names = getTra_srv.response.tarjectory.joint_trajectory.joint_names;
        for (int i = 0; i < getTra_srv.response.tarjectory.joint_trajectory.points.size(); ++i)
        {
            tra.joint_trajectory.points.push_back(getTra_srv.response.tarjectory.joint_trajectory.points[i]);
        }
    }
    return result;
}

int VisualCaptureRosFunc::motionMulti(moveit_msgs::RobotTrajectory &tra)
{
    int result = 0;

    hirop_msgs::dualRbtraject dualTra_srv;
    dualTra_srv.request.robotMotionTraject_list.resize(1);
    dualTra_srv.request.robotMotionTraject_list[0].moveGroup_name = "arm";
    dualTra_srv.request.robotMotionTraject_list[0].robot_jointTra = tra.joint_trajectory;

    if (!exeTrajectoryClient.call(dualTra_srv))
    {
        ROS_ERROR("check movePose service!");
    }
    else
    {
        if (dualTra_srv.response.is_success)
        {
            ROS_INFO_STREAM("Move to pose successfully!");
//            int flag;
//            cin >> flag;
//            if(flag)
//            {
//                moveit::
//            }
        }
        else
        {
            ROS_ERROR("Failed to move to pose!");
        }
    }

    return result;
}

int VisualCaptureRosFunc::lookPointCloud()
{
    hirop_msgs::LookPointCloud srv;
    if (lookPointCloudClient.call(srv))
    {
        if (srv.response.reuslt == 0)
            ROS_INFO("update pcl SUCCEED");
        else
            ROS_INFO("update pcl FAILED");
    }
    else
    {
        ROS_INFO("check /lookPointcloud service");
        return -1;
    }
    return srv.response.reuslt;
}

int VisualCaptureRosFunc::creatPedsDetection()
{
    pthread_t tid;
    int ret = pthread_create(&tid, NULL, bringUpPedsDetetion, (void *)this);
    if (ret == 0)
    {
        isPedsdetetion = true;
        std::cout << "创建运行行人检测成功" << std::endl;
    }
    else
    {
        isPedsdetetion = false;
        std::cout << "创建运行行人检测失败" << std::endl;
    }
    return ret;
}

bool VisualCaptureRosFunc::getPedsDetectionState()
{
    return isPedsdetetion;
}

void *VisualCaptureRosFunc::bringUpPedsDetetion(void *__this)
{
    VisualCaptureRosFunc *_this = (VisualCaptureRosFunc *)__this;
    system("rosrun hsr_pedsdetection hsr_pedsdetection.sh");
}

void VisualCaptureRosFunc::shutdownPedsDetection()
{
//    std_msgs::Bool msg;
//    msg.data = false;
//    shutDownPedsDetectionPub.publish(msg);
    system("rostopic pub -1 /switch_of_people_detect std_msgs/Bool \"data: false\"");
    this->isPedsdetetion = false;
}

/*********************/
