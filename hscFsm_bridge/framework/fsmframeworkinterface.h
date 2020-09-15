#ifndef FSMFRAMEWORKINTERFACE_H
#define FSMFRAMEWORKINTERFACE_H
#include <string>
#include <vector>
#include <hplugin.h>
#include <commonType.h>
namespace HsFsm{
enum Mode{
 Auto = 0,
 Manual = 1,
};

class FsmFramworkInterface
{
public:
    virtual void init()=0;
    virtual bool setCommand(const CmdInputData &behevior)=0;
    virtual void waitRecall()=0;
    virtual void quit()=0;
    virtual State getState()=0;
    virtual bool registerTaskList()=0;
    virtual std::string getTaskName()=0;
    virtual void debugTaskList()=0;
    virtual void setMode(Mode mode) = 0;

};

}
H_DECLARE_INTERFACE(HsFsm::FsmFramworkInterface,"HsFsm V1.0")

#endif // FSMFRAMEWORKINTERFACE_H
