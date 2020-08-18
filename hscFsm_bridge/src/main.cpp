#include <ros/ros.h>
#include <HsFsmBridge.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hsc3api_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(1);
    as.start();

//    s.start();
    HsFsmBridge bridge(n);
    bridge.start();
    ros::waitForShutdown();
    return 0;
}
