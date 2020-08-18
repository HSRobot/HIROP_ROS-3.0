#ifndef FSMFRAMEWORKINTERFACE_H
#define FSMFRAMEWORKINTERFACE_H
#include <string>
#include <vector>
namespace HsFsm {

struct CmdInputData
{
    CmdInputData() {}
    CmdInputData(const int & cmd):baheviror(cmd){}
    std::string taskName;
    int baheviror;
    int type;
    std::vector<std::string> param;
};


struct State
{
    State() {}
public:
    std::string stateName;
    std::string behevior;
    bool status;
    int type;
    std::vector<std::string> meassage;
};


class FsmFramworkInterface
{
public:
    virtual void init()=0;
    virtual bool setCommand(CmdInputData behevior)=0;
    virtual void waitRecall()=0;
    virtual void quit()=0;
    virtual State getState()=0;
    virtual bool registerTaskList()=0;
    virtual std::string getTaskName()=0;

};

}
#endif // FSMFRAMEWORKINTERFACE_H
