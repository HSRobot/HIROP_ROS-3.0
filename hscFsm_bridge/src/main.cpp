#include <ros/ros.h>
#include <HsFsmBridge.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hscfsm_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(1);
    as.start();

    std::shared_ptr<HsFsmBridge> bridge = std::make_shared<HsFsmBridge>(n);
    bridge->start();
    ros::waitForShutdown();

    return 0;
}
