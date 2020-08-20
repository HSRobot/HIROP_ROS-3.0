#pragma once
#include <ros/ros.h>
#include <hirop_msgs/taskInputCmd.h>
#include <HsTaskFramework.h>
#include <memory>
#include <atomic>
#include <queue>
#include <std_srvs/Trigger.h>
using namespace std;
using namespace HsFsm;

class HsFsmBridge
{
public:
    explicit HsFsmBridge(ros::NodeHandle &node);
    void start();

private:
    /**
     * @brief loadTask 加载任务
     */
    void loadTask();

    /**
     * @brief unloadTask 卸载任务
     */
    void unloadTask();

    /**
     * @brief initTask 初始化任务
     */
    bool initTask();

    /**
     * @brief startTask 开始任务
     */
    void startTask();

    /**
     * @brief stopTask 停止任务
     */
    void stopTask();
private:

    /**
     * @brief taskServerCmdCB 任务接收 轮询 LOOP
     * @return
     */
    bool taskServerCmdCB(hirop_msgs::taskInputCmdRequest &req, hirop_msgs::taskInputCmdResponse &res);


    /**
     * @brief getStatusCmdCB 反馈任务机的状态
     * @param req
     * @param res
     * @return
     */
    bool getStatusCmdCB(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);


    /**
     * @brief startTaskCmdCB
     * @param req
     * @param res
     * @return
     */
    bool startTaskCmdCB(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);


    /**
     * @brief stopTaskCmdCB
     * @param req
     * @param res
     * @return
     */
    bool stopTaskCmdCB(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

    /**
     * @brief cmdCbThreadLoop 任务执行 阻塞轮询
     */
    void cmdCbThreadLoop();
private:
    ros::NodeHandle nh;

    ros::ServiceServer cmdServer,getStatusServer; //cmd server call
    ros::ServiceServer startTaskServer,stopTaskServer; //cmd server call

    ros::Publisher retPub; // cmd pub

    std::shared_ptr<HsFsm::FsmFramworkInterface> framework;

    //thread callback sem
    std::shared_ptr<semaphore> sem;

    std::string cmdServerName, taskName, taskResTopName;

    std::queue<HsFsm::CmdInputData> inputCmdQue;
    bool frameExist;
    std::atomic<bool> loopStop, running;
};
