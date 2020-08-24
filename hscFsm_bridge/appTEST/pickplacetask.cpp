#include "pickplacetask.h"
#include <functional>
#include <assert.h>

#include <ros/ros.h>
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
}

void PickPlaceTask::quit()
{
    //会自动转状态为 quit
    // 用户自定义添加内容
}

/**
 * @brief PickPlaceTask::registerTaskList 用户重点关心
 * @return
 */
//
bool PickPlaceTask::registerTaskList()
{
    try{
        auto cb1 = std::bind(&PickPlaceTask::transInit2Run, this, \
                            placeholders::_1);
        //        registerTask("init","initing", cb1);


        //init --> prepare --> running --> quit
        /*
         * 必须注册事件
         * init
         * 设备关闭
         * 程序关闭
         */
        registerTask("init","initing", [&](callParm  &parm){

//            std::cout << "enter  preparing status "<<std::endl;
            // state 的状态反馈 可选
            typeCode = 0;
            taskRunStatus = true;
            setTaskState("prepare");
            notityRecall();


        });

        /*
         * 必须注册的事件
         * quit 任务退出
         * 设备关闭
         * 程序关闭
         */
        registerTask("quit", "initing", [&](callParm  &parm){
                timerRun = false;
                notityRecall();
        });


        registerTask("prepare","running", [&](callParm  &parm){
                std::cout << "parm : "<<std::endl;
                for(auto it :parm)
                {
                    std::cout<<it <<" "<<std::endl;
                }
                timerRun = true;
                setTaskState("run");

        });

        registerTask("run", "stopping", [&](callParm  &parm){
                timerRun = false;
                setTaskState("init");
        });

        /*
         * 必须注册事件
         * run 任务执行
         *
         */
        registerTask("run", "initing", [&](callParm  &parm){
                        while(ros::ok() && timerRun){
                            std::cout << "running ... "<<std::endl;
                            ros::Duration(0.5).sleep();
                        }


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
//    std::cout << "enter  preparing status "<<std::endl;

    // state 的状态反馈 可选
    typeCode = 0;
    taskRunStatus = true;

    //
    setTaskState("prepare");
    notityRecall();
}
