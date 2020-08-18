#pragma once

#include <fsm.hpp>
#include <memory>
#include <functional>
#include <fsmframeworkinterface.h>
#include <ThreadSem.h>
namespace HsFsm {

typedef int state;
typedef int behevior;
typedef std::function< void( const std::vector<std::string> &args ) > call;
typedef const std::vector<string> callParm;


class HsTaskFramework: public FsmFramworkInterface
{
public:
    HsTaskFramework();
    /**
     * @brief init
     */
    virtual void init()=0;

    /**
     * @brief setCommand
     * @param behevior
     * @return
     */
    virtual bool setCommand(CmdInputData behevior)=0;


    /**
     * @brief quit
     */
    virtual void quit()=0;

    /**
     * @brief getState
     * @param state
     * @return
     */
    virtual State getState()=0;

    /**
     * @brief registerTaskList
     * @return
     */
    virtual bool registerTaskList()=0;

    /**
     * @brief getTaskName
     * @return
     */
    virtual std::string getTaskName()=0;

protected:
    /**
     * @brief waitRecall
     */
    void waitRecall() override;


    /**
     * @brief setCommandProxy
     * @param behevior
     * @return
     */
    bool setCommandProxy(behevior behevior);

    void waitRecallProxy();

    void notityRecall();

    void registerTask(state current, behevior behevior, call call);

    State getTaskState();

    void setTaskState(state state);
protected:
    std::string taskName;

private:
    std::shared_ptr<fsm::stack> fsmStack;
    std::shared_ptr<semaphore> notitySem;

};

}
