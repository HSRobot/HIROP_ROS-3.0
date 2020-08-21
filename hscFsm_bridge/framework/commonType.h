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

    friend std::ostream &operator<<( std::ostream &out, const CmdInputData &cmd ){
        out << "--------- CmdInputData -------------"<<std::endl;
        out << "taskName: "<<cmd.taskName <<std::endl;
        out << "baheviror: "<<cmd.baheviror <<std::endl;
        out << "type: "<<cmd.type <<std::endl;
        for(auto it : cmd.param){
            out << it <<" ";
        }
        out << "----------------------"<<std::endl;
        out <<std::endl;
        return out;
    }
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
        out << "---------- GetStatus ------------"<<std::endl;
        out << "stateName: "<<t.stateName <<std::endl;
        out << "behevior: "<<t.behevior <<std::endl;
        out << "status: "<<t.status <<std::endl;
        out << "uiShowType: "<<t.uiShowType <<std::endl;
        for(auto it : t.meassage){
            out << it <<" ";
        }
        out <<std::endl;
        out << "----------------------"<<std::endl;

        return out;
    }
};

}
