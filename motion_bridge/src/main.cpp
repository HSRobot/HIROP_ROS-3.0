#include "motion.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_bridge");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    motion mt(&node_handle);
    mt.start();
//    ros::waitForShutdown();
    return 0;
}
