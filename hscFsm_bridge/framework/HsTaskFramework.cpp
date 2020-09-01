#include "HsTaskFramework.h"
using namespace HsFsm;
#include <functional>

using namespace std;
std::shared_ptr<semaphore> HsTaskFramework::notitySem = nullptr;
HsFsm::State HsTaskFramework::currentState;
HsTaskFramework::HsTaskFramework()
{
    fsmStack = std::make_shared<fsm::stack>();
    threadpool = std::make_shared<ThreadPool>(4);

}

HsTaskFramework::HsTaskFramework(ros::NodeHandle &nh, std::shared_ptr<HsTaskFramework> &fsm):typeCode(0),taskRunStatus(false)
{
    this->nh = nh;
    framework = fsm;
    fsmStack = fsm->fsmStack;
    notitySem = std::make_shared<semaphore>(fsm->taskName);

    threadpool = fsm->threadpool;
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
    notitySem->wait();
}

string HsTaskFramework::getTaskName()
{
    return taskName;
}

void HsTaskFramework::debugTaskList()
{
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
     current.Type = currentState.Type;
     current.status = currentState.status;
     current.meassage = currentState.meassage;
     current.behevior = currentState.behevior;
     return current;
}

void HsTaskFramework::setTaskState(HsFsm::state state)
{
     threadpool->enqueue(&fsm::stack::set, fsmStack.get(), std::string(state));
    // fsmStack->set(state);
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

void HsTaskFramework::setRecallState(State& state)
{
    this->currentState = state;
}

ros::NodeHandle *HsTaskFramework::getRosHandler()
{
    return &nh;
}
