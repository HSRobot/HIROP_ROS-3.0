#ifndef PICKPLACETASK_H
#define PICKPLACETASK_H

#include <HsTaskFramework.h>
namespace HsFsm {


class PickPlaceTask :public HsTaskFramework
{
public:
    PickPlaceTask(const std::string& taskName);

    virtual void init();

    virtual bool setCommand(CmdInputData behevior);

    virtual void quit();

    virtual State getState();

    virtual bool registerTaskList();

    virtual std::string getTaskName();
private:

    void transInit2Run(const std::vector<std::string> &args);
private:
    State state;
};


}
#endif // PICKPLACETASK_H
