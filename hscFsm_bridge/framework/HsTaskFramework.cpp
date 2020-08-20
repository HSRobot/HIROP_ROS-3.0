#include "HsTaskFramework.h"
using namespace HsFsm;

HsTaskFramework::HsTaskFramework():typeCode(0),taskRunStatus(false)
{
    fsmStack = std::make_shared<fsm::stack>();
}

void HsTaskFramework::waitRecall()
{
    if(notitySem ==nullptr)
        notitySem = std::make_shared<semaphore>(taskName);

    notitySem->wait();
}

string HsTaskFramework::getTaskName()
{
    return taskName;
}


bool HsTaskFramework::setCommandProxy(const HsFsm::behevior &behevior)
{
    if(!fsmStack->command(behevior))
    {
        taskRunStatus = false;
        typeCode = -1;
        notityRecall();
        return false;
    }

    return true;

}
//bool HsTaskFramework::setCommand(const CmdInputData &cmd)
//{
//    if(!fsmStack->command(cmd.baheviror))
//    {
//        taskRunStatus = false;
//        typeCode = -1;
//        notityRecall();
//        return false;
//    }

//    return true;
//}


void HsTaskFramework::notityRecall()
{
    state = getTaskState();
    notitySem->signal();
}


void HsFsm::HsTaskFramework::registerTask( HsFsm::behevior behevior, HsFsm::call call)
{

    fsmStack->on(taskName, string(behevior)) = call;
}

State HsTaskFramework::getTaskState()
{
     fsm::state ret = fsmStack->get_state();
     State current;
     current.meassage = ret.args;
     current.stateName = ret.name;
     current.Type = typeCode;
     current.status = taskRunStatus;
     return current;
}

void HsTaskFramework::setTaskState(HsFsm::state state)
{
    fsmStack->set(state);
}

void HsTaskFramework::setInitState()
{
    fsmStack->set(taskName);
}

void HsTaskFramework::setExitingAction()
{
    fsmStack->command("quit");
}
