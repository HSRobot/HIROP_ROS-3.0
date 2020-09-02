
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <hscloudcollection_bridge_node.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hsr_impedance");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    HsCloudCollectionBridge bridge(n);
    bridge.start();
    return 0;
}


//redisControl red;
//assert(0 == red.connect("127.0.0.1", 6379));
////    for(int i = 0 ; i < 55;  i++){
//for(;;){
//    vector<double> temp{1, 2,3,4,5,6};
//    red.setpublishForceData(temp);
//    if(0!= red.publish())
//    {
//        perror("publish error");
//        break;
//    }
//    std::this_thread::sleep_for(std::chrono::seconds(1));
//    std::cout << "publish ok "<<std::endl;
//}
