#include <MotionCoorServer.h>
#include <hirop_msgs/ImpedenceAdjustStart.h>
#include <hirop_msgs/ImpedenceAdjustStop.h>
#include <boost/thread.hpp>
MotionCoorServer::MotionCoorServer(ros::NodeHandle &nh)
{
    this->nh = nh;
    motionCoorSet_server = nh.advertiseService("/force_bridge/motionCoorStart",  &MotionCoorServer::motionCoorStartCB, this);
    motionCoorShutDown_server = nh.advertiseService("/force_bridge/motionCoorStop",  &MotionCoorServer::motionCoorStopCB, this);
//    pub_rob  =nh.subscribe("/plan/joint_path_command",1, &MotionCoorServer::pathTrajPointCB, this);

    motionImpedenceStartCli = nh.serviceClient<hirop_msgs::ImpedenceAdjustStart>("/force_bridge/impedenceStart");
    motionImpedenceStopCli = nh.serviceClient<hirop_msgs::ImpedenceAdjustStop>("/force_bridge/impedenceStop");

    isRunning = false;
    stop = false;
}

void MotionCoorServer::start()
{
    ROS_INFO_STREAM("MotionCoorServer start....");
    ros::MultiThreadedSpinner ms(1);
    ms.spin();
}

bool MotionCoorServer::motionCoorStartCB(hirop_msgs::motionCoorStartRequest &req, hirop_msgs::motionCoorStartResponse &res)
{
    pub_rob  =nh.subscribe("/plan/joint_path_command",1, &MotionCoorServer::pathTrajPointCB, this);

    if(isRunning){
        res.result = false;
        return true;
    }

    isRunning = true;
    stop = false;
    boost::function0<void> f1 = boost::bind(&MotionCoorServer::threadRun, this);
    boost::thread t1(f1);
    t1.detach();
    res.result = true;
    return true;
}

bool MotionCoorServer::motionCoorStopCB(hirop_msgs::motionCoorStopRequest &req, hirop_msgs::motionCoorStopResponse &res)
{
    stop = true;
    isRunning = false;
    hirop_msgs::ImpedenceAdjustStop srv;
    motionImpedenceStopCli.call(srv);
    res.result = srv.response.result;
    return true;
}


void MotionCoorServer::pathTrajPointCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
//    std::queue<trajectory_msgs::JointTrajectory> temp;
    jointTrajQueue = std::queue<trajectory_msgs::JointTrajectory>();
    jointTrajQueue.push(*msg);
    pub_rob.shutdown();
    condition.notify_all();
}

void MotionCoorServer::threadRun()
{
    while(!stop && ros::ok())
    {
        {
            ROS_ERROR_STREAM("threadRun waiting ............ ");

            boost::mutex mtx;
            boost::unique_lock<boost::mutex> lock(mtx);
            condition.wait(lock);
        }

        if(jointTrajQueue.size() > 0)
        {
            trajectory_msgs::JointTrajectory jtraj = jointTrajQueue.front();
            hirop_msgs::ImpedenceAdjustStart msg;
            msg.request.trajectPath = jtraj;
            msg.request.mode = 2;
            bool ret = motionImpedenceStartCli.call(msg);
            if(!ret){
                break ;
                ROS_ERROR_STREAM("ERROR call ret: "<< ret);

            }
            pub_rob  =nh.subscribe("/plan/joint_path_command",1, &MotionCoorServer::pathTrajPointCB, this);

            int retInt = msg.response.result;
            ROS_ERROR_STREAM("response.result: "<< retInt);

        }

        ros::Duration(0.1).sleep();
    }
    stop = true;
    isRunning = false;
    ROS_ERROR_STREAM("ERROR stop ............ ");
}
