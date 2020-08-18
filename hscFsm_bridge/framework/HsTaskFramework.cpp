#include "HsTaskFramework.h"
using namespace HsFsm;

HsTaskFramework::HsTaskFramework()
{
    fsmStack = std::make_shared<fsm::stack>();
}

void HsTaskFramework::waitRecall()
{
   waitRecallProxy();
}

void HsTaskFramework::waitRecallProxy()
{
    if(notitySem ==nullptr)
        notitySem = std::make_shared<semaphore>(taskName);

    notitySem->wait();
}

bool HsTaskFramework::setCommandProxy(HsFsm::behevior behevior)
{
    fsmStack->command(behevior);

}

void HsTaskFramework::notityRecall()
{
    notitySem->signal();
}


void HsFsm::HsTaskFramework::registerTask(HsFsm::state current, HsFsm::behevior behevior, HsFsm::call call)
{

    fsmStack->on(current,behevior) = call;
}

State HsTaskFramework::getTaskState()
{
     fsm::state ret = fsmStack->get_state();
     State current;
     current.meassage = ret.args;
     current.stateName = ret.name;
     return current;
}

void HsTaskFramework::setTaskState(HsFsm::state state)
{
    fsmStack->set(state);
}
