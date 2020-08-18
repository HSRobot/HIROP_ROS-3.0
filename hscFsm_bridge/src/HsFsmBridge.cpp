#include <HsFsmBridge.h>
#include <pickplacetask.h>
#include <future>
#include <hirop_msgs/taskCmdRet.h>
using namespace HsFsm;

HsFsmBridge::HsFsmBridge(ros::NodeHandle &node):nh(node),loopStop(false)
{
    nh.param("TaskServer",cmdServerName, std::string("TaskServerDefault"));
    nh.param("TaskName", taskName, std::string("pickplace"));
    nh.param("taskResTopName", taskResTopName, std::string("pickplace"));

    sem = std::make_shared<semaphore>(cmdServerName);

    cmdServer = nh.advertiseService(cmdServerName, &HsFsmBridge::taskServerCmdCB,this);

    retPub = nh.advertise<hirop_msgs::taskCmdRet>(taskResTopName,1);
    loadTask();
    cmdCbThreadLoop();
}

void HsFsmBridge::start()
{
    ROS_INFO_STREAM("ROS Task event start ");
    ros::MultiThreadedSpinner sp(1);
    sp.spin();
}


bool HsFsmBridge::taskServerCmdCB(hirop_msgs::taskInputCmdRequest &req, hirop_msgs::taskInputCmdResponse &res)
{
     CmdInputData inputCmd;
     inputCmd.taskName =  req.taskName;
     inputCmd.baheviror = req.behavior;
     inputCmd.param = req.param;
     inputCmd.type = req.type;
     inputCmdQue.push(inputCmd);
     sem->signal();
     return true;
}

void HsFsmBridge::cmdCbThreadLoop()
{
    loopStop = false;
    auto loop = std::async(std::launch::async, [&]{
        while(!loopStop){
            if(inputCmdQue.empty())
                sem->wait();
            CmdInputData temp = inputCmdQue.front();
            inputCmdQue.pop();
            framework->setCommand(temp);
        }
    });

    auto loop2 = std::async(std::launch::async,[&](){
        while(!loopStop){
           framework->waitRecall();
           State ret = framework->getState();
           if(!ret.status)
               framework->quit();

           if(ret.type == 1){
               hirop_msgs::taskCmdRet RetMsg;
               RetMsg.ret = 0;
               retPub.publish(RetMsg);
           }
        }
    });


}


void HsFsmBridge::loadTask()
{
    taskName;
    framework = std::make_shared<HsFsm::PickPlaceTask>("pickplace");
    std::cout <<framework->getTaskName()<<std::endl;
}

void HsFsmBridge::unloadTask()
{
    framework.reset();
}

void HsFsmBridge::initTask()
{
    bool ret =framework->registerTaskList();
    if(!ret){
        ROS_ERROR_STREAM("registerTaskList error");
        return ;
    }

    framework->init();

}

void HsFsmBridge::startTask()
{
    loopStop = false;
    CmdInputData inputCmd;
    inputCmd.type = 'start';
    framework->setCommand(inputCmd);
}

void HsFsmBridge::stopTask()
{
    loopStop = true;
    framework->quit();
}




