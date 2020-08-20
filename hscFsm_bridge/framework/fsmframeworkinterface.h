#ifndef FSMFRAMEWORKINTERFACE_H
#define FSMFRAMEWORKINTERFACE_H
#include <string>
#include <vector>
namespace HsFsm {

struct CmdInputData
{
    CmdInputData() {}
    CmdInputData(const std::string & cmd):baheviror(cmd){}
    std::string taskName;
    std::string baheviror;
    int type;
    std::vector<std::string> param;
};


struct State
{
    State():status(false),Type(0),uiShowType(0) {}
public:
    std::string stateName;
    std::string behevior;
    bool status;
    int uiShowType;
    int Type;
    std::vector<std::string> meassage;

    friend std::ostream &operator<<( std::ostream &out, const State &t ){
        out << "----------------------"<<std::endl;
        out << "stateName: "<<t.stateName <<std::endl;
        out << "behevior: "<<t.behevior <<std::endl;
        out << "status: "<<t.status <<std::endl;
        out << "uiShowType: "<<t.uiShowType <<std::endl;
        for(auto it : t.meassage){
            out << it <<" ";
        }
        out << "----------------------"<<std::endl;

        return out;
    }
};

/*
 * errorType -1 找不到 相关的行为
 *
 */

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

};

}
#endif // FSMFRAMEWORKINTERFACE_H
