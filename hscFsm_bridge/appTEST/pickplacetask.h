#ifndef PICKPLACETASK_H
#define PICKPLACETASK_H

#include <HsTaskFramework.h>
namespace HsFsm {


class PickPlaceTask :public HsTaskFramework
{
public:
    PickPlaceTask(const string &taskName);

    virtual void init();

    virtual bool setCommand(const CmdInputData &cmd);

    virtual void quit();

    virtual State getState();

    virtual bool registerTaskList();

//    virtual std::string getTaskName();
private:

    void transInit2Run(const std::vector<std::string> &args);
private:
    std::map<std::string, int> taskMap;
};


}
#endif // PICKPLACETASK_H
