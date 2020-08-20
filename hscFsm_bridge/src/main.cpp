#include <ros/ros.h>
#include <HsFsmBridge.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hsc3api_bridge");
    ros::NodeHandle n;
    ros::AsyncSpinner as(1);
    as.start();
//    std::string str = "init1";
//    std::cout <<std::atoi(str.c_str())<<std::endl;

//    std::cout <<'init'<<std::endl;
//    std::cout <<'init1'<<std::endl;

    std::shared_ptr<HsFsmBridge> bridge = std::make_shared<HsFsmBridge>(n);
    bridge->start();
    ros::waitForShutdown();

    return 0;
}
