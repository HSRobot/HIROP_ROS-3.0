#include "HsTaskFramework.h"
using namespace HsFsm;
#include <functional>
HsTaskFramework::HsTaskFramework()
{
    fsmStack = std::make_shared<fsm::stack>();
    notitySem = std::make_shared<semaphore>(taskName);
}

HsTaskFramework::HsTaskFramework(std::shared_ptr<HsTaskFramework> &fsm):typeCode(0),taskRunStatus(false)
{
    framework = fsm;
    fsmStack = fsm->fsmStack;
    notitySem = fsm->notitySem;
    threadpool = std::make_shared<ThreadPool>(4);
}

void HsTaskFramework::init()
{

    framework->init();
    setInitState();

}

bool HsTaskFramework::setCommand(const CmdInputData &cmd)
{
    //
//    framework->setCommand(cmd);
    setCommandProxy(cmd);
}

void HsTaskFramework::quit()
{
    framework->quit();
    setExitingAction();

}

State HsTaskFramework::getState()
{
    return getTaskState();
}

bool HsTaskFramework::registerTaskList()
{
    return framework->registerTaskList();
}

void HsTaskFramework::waitRecall()
{
//    if(notitySem ==nullptr)
//        notitySem = std::make_shared<semaphore>(taskName);

    notitySem->wait();
}

string HsTaskFramework::getTaskName()
{
    return taskName;
}

void HsTaskFramework::debugTaskList()
{
//    ostream os;
//    os << fsmStack->debug(os);
    std::cout <<fsmStack.get();
}

/**
 * @brief HsTaskFramework::setCommandProxy 指令运行代理函数
 * @param behevior
 * @return
 */
bool HsTaskFramework::setCommandProxy(const CmdInputData &cmd)
{
    if(!fsmStack->findFuntionalIsVaild(cmd.baheviror.c_str()))
    {
        taskRunStatus = false;
        typeCode = -1;
        notityRecall();
        return false;
    }

    std::cout << cmd;
    threadpool->enqueue(&fsm::stack::commandThread, fsmStack.get(), cmd.baheviror.c_str(),cmd.param);
    return true;

}


void HsTaskFramework::notityRecall()
{
    currentState = getTaskState();
    notitySem->signal();
}


void HsFsm::HsTaskFramework::registerTask( HsFsm::state state, HsFsm::action behevior, HsFsm::call call)
{

    fsmStack->on(string(state), string(behevior)) = call;
}

State HsTaskFramework::getTaskState()
{
     fsm::state ret = fsmStack->get_state();
     State current;
     current.meassage = ret.args;
     current.stateName = ret.name;
     current.Type = typeCode;
     current.status = taskRunStatus;
     current.meassage = recallMessage;
     return current;
}

void HsTaskFramework::setTaskState(HsFsm::state state)
{
    fsmStack->set(state);
}

void HsTaskFramework::setInitState()
{
    threadpool->Start();
    fsmStack->set("init");
}

void HsTaskFramework::setExitingAction()
{
    fsmStack->set("quit");

    threadpool->Stop();
}

void HsTaskFramework::setRecallState(State state)
{
    typeCode = state.Type;
    taskRunStatus = state.status;
    recallMessage = state.meassage;
    this->currentState = state;
}
