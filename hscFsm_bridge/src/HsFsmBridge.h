#pragma once
#include <ros/ros.h>
#include <hirop_msgs/taskInputCmd.h>
#include <HsTaskFramework.h>
#include <memory>
//#include <ThreadSem.h>
#include <queue>
using namespace std;
using namespace HsFsm;

class HsFsmBridge
{
public:
    explicit HsFsmBridge(ros::NodeHandle &node);
    void start();

private:
    /**
     * @brief loadTask
     */
    void loadTask();

    /**
     * @brief unloadTask
     */
    void unloadTask();

    /**
     * @brief initTask
     */
    void initTask();

    /**
     * @brief startTask
     */
    void startTask();

    /**
     * @brief stopTask
     */
    void stopTask();
private:

    /**
     * @brief taskServerCmdCB
     * @return
     */
    bool taskServerCmdCB(hirop_msgs::taskInputCmdRequest &req, hirop_msgs::taskInputCmdResponse &res);

    void cmdCbThreadLoop();
private:
    ros::NodeHandle nh;

    ros::ServiceServer cmdServer; //cmd server call

    ros::Publisher retPub; // cmd pub

    std::shared_ptr<HsFsm::FsmFramworkInterface> framework;

    //thread callback sem
    std::shared_ptr<semaphore> sem;

    std::string cmdServerName, taskName, taskResTopName;

    std::queue<HsFsm::CmdInputData> inputCmdQue;
    bool loopStop, running;
};
