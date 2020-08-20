#pragma once

#include <fsm.hpp>
#include <memory>
#include <functional>
#include <fsmframeworkinterface.h>
#include <ThreadSem.h>
namespace HsFsm {

typedef const char* state;
typedef string behevior;
typedef std::function< void( const std::vector<std::string> &args ) > call;
typedef const std::vector<string> callParm;

//#define TASK(T) std::string(T)


class HsTaskFramework: public FsmFramworkInterface
{
public:
    HsTaskFramework();

    /**
     * @brief init 子任务的初始化
     */
    virtual void init()=0;

    /**
     * @brief setCommand 子任务 的相关行为
     * @param behevior
     * @return
     */
    virtual bool setCommand(const CmdInputData &cmd)=0;


    /**
     * @brief quit  子任务的退出
     */
    virtual void quit()=0;

    /**
     * @brief getState 获取子任务的当前状态
     * @param state
     * @return
     */
    virtual State getState()=0;

    /**
     * @brief registerTaskList 子任务的绑定事件的注册
     * @return
     */
    virtual bool registerTaskList()=0;



protected:
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
     * @brief setCommandProxy
     * @param behevior
     * @return
     */
    bool setCommandProxy(const behevior &behevior);


    /**
     * @brief notityRecall 结果状态反馈
     */
    void notityRecall();


    void registerTask(behevior behevior, call call);


    /**
     * @brief getTaskState 会获取当前的任务状态 以及 错误码 任务名称
     * @return
     */
    State getTaskState();


    /**
     * @brief setTaskState
     * @param state
     */
    void setTaskState(state state);

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
    State state;

private:
    std::shared_ptr<fsm::stack> fsmStack;
    std::shared_ptr<semaphore> notitySem;

};

}
