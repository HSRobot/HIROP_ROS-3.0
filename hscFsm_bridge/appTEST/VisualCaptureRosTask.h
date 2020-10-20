#ifndef VISUALCAPTUREROSTASK_H
#define VISUALCAPTUREROSTASK_H

#include <HsTaskFramework.h>
#include "VisualCaptureRosFunc.h"
#include <ros/ros.h>
#include <functional>
#include <assert.h>
#include <iostream>


using namespace std;

namespace HsFsm{

class VisualCaptureRosTask : public HsTaskFramework
{
public:

    VisualCaptureRosTask(const string & taskname);

    ~VisualCaptureRosTask();

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
    std::string detect_object;
    VisualCaptureRosFunc *VCRF;
    std::string isTest;
    vector<string > ErrInfoList;
    int index_errinfo=-1;

private:

    //状态运行标志位
    atomic<bool> isIniting;
    atomic<bool> isPreparing;
    atomic<bool> isDetecting;
    atomic<bool> isLooking;
    atomic<bool> isPicking;
    atomic<bool> isPlacing;
    atomic<bool> isError;
    atomic<bool> isExiting;
    atomic<bool> isForceQuit;
    

    //状态行为函数
    void init_initing(const std::vector<std::string> &args);
    void init_quiting(const std::vector<std::string> &args);
    void init_starting(const std::vector<std::string> &args);

    void prepare_initing(const std::vector<std::string> &args);
    void prepare_quiting(const std::vector<std::string> &args);
    void transPrepare2Detection(const std::vector<std::string> &args);
    void transPrepare2Exit(const std::vector<std::string> &args);

    void look_initing(const std::vector<std::string> &args);
    void test_initing(const std::vector<std::string> &args);
    void look_quiting(const std::vector<std::string> &args);

    void detection_initing(const std::vector<std::string> &args);
    void detection_quiting(const std::vector<std::string> &args);
    void transDetection2Exit(const std::vector<std::string> &args);
    
    void pick_initing(const std::vector<std::string> &args);
    void pick_quiting(const std::vector<std::string> &args);
    void transPick2Exit(const std::vector<std::string> &args);

    void place_initing(const std::vector<std::string> &args);
    void place_quiting(const std::vector<std::string> &args);
    void transPlace2Exit(const std::vector<std::string> &args);

    void error_initing(const std::vector<std::string> &args);
    void error_quiting(const std::vector<std::string> &args);

    void exit_initing(const std::vector<std::string> &args);
    void exit_quiting(const std::vector<std::string> &args);
    void transExit2Init(const std::vector<std::string> &args);

    void publishStateMsg(bool status, string behaviour, string message);

    int toDetection(int where);

};
}

#endif //VISUALCAPTUREROSTASK_H
