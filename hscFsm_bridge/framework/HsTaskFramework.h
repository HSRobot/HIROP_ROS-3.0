#pragma once

#include <fsm.hpp>
#include <memory>
#include <functional>
#include <fsmframeworkinterface.h>
#include <ThreadSem.h>
#include <ThreadPool.h>
#include <ros/ros.h>
#include <hplugin.h>
namespace HsFsm {

typedef const char* state;
typedef const char* action;
typedef string behevior;
typedef std::function< void( const std::vector<std::string> &args ) > call;
typedef const std::vector<string> callParm;

//#define TASK(T) std::string(T)



class HsTaskFramework: public FsmFramworkInterface
{
public:
    HsTaskFramework();

    HsTaskFramework(ros::NodeHandle &nh, std::shared_ptr<HsTaskFramework> &fsm);

    /**
     * @brief init 子任务的初始化
     */
    virtual void init();


    /**
     * @brief registerTaskList 子任务的绑定事件的注册
     * @return
     */
    virtual bool registerTaskList();


    /**
     * @brief quit  子任务的退出
     */
    virtual void quit();

/*********************************************************************/

public:
    /**
     * @brief getState 获取子任务的当前状态
     * @param state
     * @return
     */
     State getState();

     /**
      * @brief setCommand 子任务 的相关行为
      * @param behevior
      * @return
      */
     bool setCommand(const CmdInputData &cmd);

     /**
      * @brief waitRecall 等待任务完成的响应 阻塞式
      */
     void waitRecall() override;

     /**
      * @brief getTaskName 子任务的任务名称
      * @return
      */
     std::string getTaskName() override;


     /**
      * @brief debugTaskList
      */
     void debugTaskList() override;

protected:
    /**
     * @brief setMode
     * @param mode
     */
    virtual void setMode(Mode mode);

    /**
     * @brief setTaskState
     * @param state
     */
    void setTaskState(HsFsm::state state);

    /**
     * @brief setRecallState 重置反馈结果状态
     * @param state
     */
    void setRecallState(State &state);

    /**
     * @brief getRosHandler
     * @return
     */
    ros::NodeHandle* getRosHandler();

    /**
     * @brief notityRecall 结果状态反馈
     */
    void notityRecall();

    /**
     * @brief registerTask
     * @param state
     * @param behevior
     * @param call
     */
    void registerTask( HsFsm::state state, HsFsm::action behevior, HsFsm::call call);

private:
    /**
     * @brief setCommandProxy
     * @param behevior
     * @return
     */
    bool setCommandProxy(const CmdInputData &cmd);

    /**
     * @brief getTaskState 会获取当前的任务状态 以及 错误码 任务名称
     * @return
     */
    State getTaskState();

    /**
     * @brief setInitState
     */
    void setInitState();

    /**
     * @brief setExitingAction 程序退出
     */
    void setExitingAction();




protected:
    std::string taskName;
    int typeCode;
    bool taskRunStatus;
    std::vector<string> recallMessage;
private:
    std::shared_ptr<fsm::stack> fsmStack;
    std::shared_ptr<semaphore> notitySem;

    std::shared_ptr<ThreadPool> threadpool;
    static HsFsm::State currentState;
private:
    std::weak_ptr<HsTaskFramework> framework;
    ros::NodeHandle nh;
    Mode mode;

};
H_DECLARE_PLUGIN(FsmFramworkInterface)
}





