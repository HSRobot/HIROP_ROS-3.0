#include "pickplacetask.h"
#include <functional>
#include <assert.h>
using namespace HsFsm;
typedef std::pair<string, int> elment;

PickPlaceTask::PickPlaceTask(const string &taskName)
{
    this->taskName = taskName;
}



void PickPlaceTask::init()
{
    // 自动转为 init 状态
    // 用户自定义添加内容
    setInitState();
}

bool PickPlaceTask::setCommand(const CmdInputData &cmd)
{
    return setCommandProxy(cmd.baheviror);
}


void PickPlaceTask::quit()
{
    //会自动转状态为 quit
    // 用户自定义添加内容
    setExitingAction();
}


State PickPlaceTask::getState()
{
    return state;
}



bool PickPlaceTask::registerTaskList()
{
    try{
        auto cb1 = std::bind(&PickPlaceTask::transInit2Run, this, \
                            placeholders::_1);

        /*
         * 必须注册事件
         * init
         * 设备关闭
         * 程序关闭
         */
        registerTask("initing", cb1);

        /*
         * 必须注册事件
         * run 任务执行
         *
         */
        registerTask("running", [&](callParm  &parm){
                std::cout << "running ... "<<std::endl;
                while(1);
                    });

        /*
         * 必须注册的事件
         * quit 任务退出
         * 设备关闭
         * 程序关闭
         */
        registerTask("quit", [&](callParm  &parm){
                std::cout << "exiting ... "<<std::endl;
                notityRecall();
        });


    }catch(std::exception &e)
    {
        std::cout  << e.what()<<std::endl;
        assert(-1);
        return false;
    }


    return true;
}


void PickPlaceTask::transInit2Run(const std::vector<string> &args)
{
    std::cout << "enter  preparing status "<<std::endl;

    // state 的状态反馈 可选
    typeCode = 0;
    taskRunStatus = true;
    notityRecall();
}
