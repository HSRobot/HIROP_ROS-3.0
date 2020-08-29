#include <gtest/gtest.h>
#include <pickplacetask.h>
#include <fsmframeworkinterface.h>
#include <memory>
#include <ros/ros.h>
using namespace HsFsm;
class MYTestClass : public testing::Test {
protected:
    virtual void SetUp() {
        // some initialization of testing
        ros::NodeHandle nh;
        std::shared_ptr<HsTaskFramework> ptr = std::make_shared<HsFsm::PickPlaceTask>("VoiceCtlRob");
        framework = std::make_shared<HsFsm::HsTaskFramework>(nh,ptr);

    }
    virtual void TearDown() {

    }
protected:
    std::shared_ptr<FsmFramworkInterface> framework;
};

TEST_F(MYTestClass, my_test_1) {
   // some test operation
    ASSERT_EQ(true, framework->registerTaskList());
    framework->init();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "hscfsm_bridge");
    testing::InitGoogleTest(&argc, argv);// gtest 的初始化

   return RUN_ALL_TESTS();//调用 GTest 测试用例
}
