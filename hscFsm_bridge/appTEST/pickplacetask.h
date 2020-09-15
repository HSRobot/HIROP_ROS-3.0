#pragma once

#include <HsTaskFramework.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
using namespace HsFsm;


class PickPlaceTask :public HsTaskFramework
{
public:
    PickPlaceTask();
    PickPlaceTask(const string &taskName);

    /**
     * @brief init
     */
    virtual void init();

    /**
     * @brief quit
     */
    virtual void quit();

    /**
     * @brief registerTaskList
     * @return
     */
    virtual bool registerTaskList();

private:

    void transInit2Run(const std::vector<std::string> &args);
    void shake_hand_order_CallBack(const std_msgs::Int16::ConstPtr & msg);
private:
    ros::Subscriber voice_order_sub;
    std::map<std::string, int> taskMap;

private:
    bool timerRun;
    int voice_order;
};




