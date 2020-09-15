#pragma once
#include <boost/shared_ptr.hpp>
#include <HsTaskFramework.h>
using namespace HsFsm;

enum Task{
   PickPlace =1,
   Unkown = 0,
};

class HsTaskFsmFactory
{
private:
    HsTaskFsmFactory();
public:
    /**
     * @brief createByTest
     * @param nh
     * @param type
     * @return
     */
    static std::shared_ptr<FsmFramworkInterface> createByTest(ros::NodeHandle &nh,
                                                     int type);
    static std::shared_ptr<FsmFramworkInterface> createByPlugin(ros::NodeHandle &nh,
                                                     const std::string &taskName);

    static std::vector<std::string> getClassPluginList();


};
