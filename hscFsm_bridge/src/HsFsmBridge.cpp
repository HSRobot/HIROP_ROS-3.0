#include <HsFsmBridge.h>
#include <pickplacetask.h>
#include <future>
#include <hirop_msgs/taskCmdRet.h>
using namespace HsFsm;

HsFsmBridge::HsFsmBridge(ros::NodeHandle &node):nh(node),loopStop(false),frameExist(false),running(false)
{
    nh.param("TaskServer",cmdServerName, std::string("TaskServerDefault"));
    nh.param("TaskName", taskName, std::string("pickplace"));
    nh.param("taskResTopName", taskResTopName, std::string("pickplacePub"));

    sem = std::make_shared<semaphore>(cmdServerName);

    cmdServer = nh.advertiseService(cmdServerName, &HsFsmBridge::taskServerCmdCB,this);

    getStatusServer = nh.advertiseService("getStatusServer", &HsFsmBridge::getStatusCmdCB,this);

    startTaskServer = nh.advertiseService("startTaskServer", &HsFsmBridge::startTaskCmdCB,this);

    stopTaskServer = nh.advertiseService("stopTaskServer", &HsFsmBridge::stopTaskCmdCB,this);

    retPub = nh.advertise<hirop_msgs::taskCmdRet>(taskResTopName,1);

    // 加载子任务
    loadTask();

    // 启动循环判断
    cmdCbThreadLoop();

    // 初始化子任务
    initTask();
    running = true;
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

bool HsFsmBridge::getStatusCmdCB(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    if(framework == nullptr)
        return false;

    HsFsm::State  cur = framework->getState();
    std::cout << cur ;
    return true;
}

bool HsFsmBridge::startTaskCmdCB(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    if(!frameExist)
        return false;

    if(!running)
        running = true;
    else{
        ROS_ERROR_STREAM("Task is running... ");
        return false;
    }
    // 初始化子任务
    initTask();
    // 启动循环判断
    cmdCbThreadLoop();
    res.success = 1;
    return true;
}

bool HsFsmBridge::stopTaskCmdCB(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{

    stopTask();
    running = false;
    ROS_INFO_STREAM("ROS Task event stopTaskCmd ");
    res.success = 1;
    return true;
}

void HsFsmBridge::cmdCbThreadLoop()
{
    if(framework ==nullptr)
        return ;

    ROS_INFO_STREAM("ROS Task event cmdCbThreadLoop start ");

    loopStop = false;
    if(!inputCmdQue.empty())
    {
        std::queue<HsFsm::CmdInputData> empty;
        inputCmdQue.swap(empty);
    }


//    auto loop = std::async(std::launch::async, [&]{
    std::thread t1([&]{
        while(!loopStop){
            if(inputCmdQue.size() == 0){
                sem->wait();
            }

            std::cout <<"inputCmdQue.size: "<< inputCmdQue.size()<<std::endl;
            CmdInputData temp = inputCmdQue.front();
            inputCmdQue.pop();
            framework->setCommand(temp);
        }
    });

//    auto loop2 = std::async(std::launch::async,[&](){
    std::thread t2([&]{
        while(!loopStop){
           framework->waitRecall();
           State ret = framework->getState();

           hirop_msgs::taskCmdRet RetMsg;

           if(!ret.status){
               switch (ret.Type){
                   case -2:{
                       ROS_ERROR_STREAM("#E -2 reserved");
                       break;
                    }
                   case -1:{
                          ROS_ERROR_STREAM("#E -1 Action can't  found! Please Check");
                          break;
                   }
                    case -3:{
                          // 严重错误 退出.
                          ROS_ERROR_STREAM("#E -2 Action can't  found! Please Check");
                          framework->quit();
                          break;
                    }
                }
               RetMsg.ret = ret.Type;
               retPub.publish(RetMsg);
               continue;
           }

           RetMsg.ret = ret.status;
           RetMsg.task = ret.stateName;
           RetMsg.behevior = ret.behevior;
//           RetMsg.message = ret.meassage.at(0);
           retPub.publish(RetMsg);
           ROS_DEBUG_STREAM("#action ok");

        }
    });

    t1.detach();
    t2.detach();
}


void HsFsmBridge::loadTask()
{
    taskName;
    framework = std::make_shared<HsFsm::PickPlaceTask>("pickplace");
    std::cout <<framework->getTaskName()<<std::endl;
    frameExist = true;
}

void HsFsmBridge::unloadTask()
{
    framework.reset();
}

bool HsFsmBridge::initTask()
{
    bool ret =framework->registerTaskList();
    if(!ret){
        ROS_ERROR_STREAM("#E -3 registerTaskList error");
        return false;
    }

    framework->init();

}

void HsFsmBridge::startTask()
{
    loopStop = false;
    CmdInputData inputCmd;
    inputCmd.type = 0;
    framework->setCommand(inputCmd);
}

void HsFsmBridge::stopTask()
{
    loopStop = true;
    framework->quit();
}




