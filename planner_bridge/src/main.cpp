#include "planner_bridge/planning_bridge.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trajectory_planner");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner s(4);
    s.start();
    PlanningBridge pb(nh);
    ros::waitForShutdown();
    return 0;
}
