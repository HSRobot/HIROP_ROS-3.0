#include "pickplacetask.h"
#include <functional>
using namespace HsFsm;

PickPlaceTask::PickPlaceTask(const std::string &taskName)
{
    this->taskName = taskName;
}


void PickPlaceTask::init()
{

}

bool PickPlaceTask::setCommand(CmdInputData behevior)
{
    setCommandProxy(behevior.baheviror);
}


void PickPlaceTask::quit()
{
}

State PickPlaceTask::getState()
{
    return state;
}

bool PickPlaceTask::registerTaskList()
{
    auto cb1 = std::bind(&PickPlaceTask::transInit2Run, this, \
                        placeholders::_1);
    registerTask('init', 'running', cb1);

    registerTask('running', 'quit', [&](callParm  &parm){
        this->setTaskState('quit');
    });
}

std::string PickPlaceTask::getTaskName()
{
    return taskName;
}

void PickPlaceTask::transInit2Run(const std::vector<string> &args)
{

}
