#include <iostream>
#include <hscloudorder_bridge.h>
#include <ros/ros.h>
using namespace std;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "hscfsm_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(1);
    as.start();

    HsCloudOrderBridge bridge(n);
    bridge.start();
    return 0;
}
