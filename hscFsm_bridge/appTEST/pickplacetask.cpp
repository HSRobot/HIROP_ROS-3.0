#include "pickplacetask.h"
#include <functional>
#include <assert.h>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
using namespace HsFsm;
typedef std::pair<string, int> elment;

PickPlaceTask::PickPlaceTask()
{
    this->taskName = "PickPlaceTask";
}


void PickPlaceTask::init()
{
    voice_order_sub = getRosHandler()->subscribe("/robot_order", 1, &PickPlaceTask::shake_hand_order_CallBack, this);
    // 自动转为 init 状态
    // 用户自定义添加内容
}

void PickPlaceTask::quit()
{
    //会自动转状态为 quit
    // 用户自定义添加内容
}

/**.,
 * @brief PickPlaceTask::registerTaskList 用户重点关心
 * @return
 */
//
bool PickPlaceTask::registerTaskList()
{
    try{
        auto cb1 = std::bind(&PickPlaceTask::transInit2Run, this, \
                            placeholders::_1);


        //init --> prepare --> running --> quit
        /*
         * 必须注册事件
         * init
         * 设备关闭
         * 程序关闭
         */
        registerTask("init","initing", [&](callParm  &parm){

            std::cout << "enter init state, behaviour: initing" << std::endl;
            // state 的状态反馈 可选
            typeCode = 0;
            taskRunStatus = true;

            State sta;
            sta.meassage = {"enter init state, behaviour: initing"};
            setRecallState(sta);
            notityRecall();

            setTaskState("prepare");

        });

        /*
         * 必须注册的事件
         * quit 任务退出
         * 设备关闭
         * 程序关闭
         */
        registerTask("quit", "initing", [&](callParm  &parm){
                std::cout << "enter quit state, behaviour: initing" << std::endl;
                timerRun = false;

                //stop_motion

                notityRecall();
        });

        //prepare initing quiting
        registerTask("prepare","initing", [&](callParm  &parm){
                std::cout << "enter prepare state, behaviour: initing" << std::endl;
                //                move()
        });



        registerTask("prepare","running", [&](callParm  &parm){
                std::cout << "enter prepare state, behaviour: running" << std::endl;
//                std::cout << "parm : "<<std::endl;
//                for(auto it :parm)
//                {
//                    std::cout<<it <<" "<<std::endl;
//                }

                timerRun = true;

                State sta;
                sta.meassage = {" "};
                sta.status = 1;

                std::cout << "Ready to shake_hand or grasp_toy!Please have a choice!" << std::endl;
                /*******/
                sta.meassage = {"enter prepare state, behaviour: running"};
                setRecallState(sta);
                notityRecall();

                setTaskState("run");


        });

        registerTask("run", "stopping", [&](callParm  &parm){
                timerRun = false;
                notityRecall();

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
//                         setTaskState("quit");

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

void PickPlaceTask::shake_hand_order_CallBack(const std_msgs::Int16::ConstPtr & msg)
{
    voice_order = msg->data;
    std::cout << "voice_order: " << voice_order << std::endl;
    if (voice_order == 0)
        setTaskState("init");
}

H_EXPORT_PLUGIN(PickPlaceTask, "PickPlaceTask" , "1.0")
