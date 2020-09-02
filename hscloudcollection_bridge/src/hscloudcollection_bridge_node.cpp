#include <ros/ros.h>
#include <hscloudcollection_bridge_node.h>


HsCloudCollectionBridge::HsCloudCollectionBridge(ros::NodeHandle &nh):ip("127.0.0.1"),port(6379)
{
    this->nh = nh;
    redisCon = boost::make_shared<redisControl>();
}

void HsCloudCollectionBridge::start()
{
    ros::MultiThreadedSpinner sp(1);
    sp.spin();
}

void HsCloudCollectionBridge::connectCloudSrvCB()
{
    int ret = redisCon->connect(ip, port);

}

void HsCloudCollectionBridge::setJsonKeySrvCB()
{
//    redisCon->setJsonkey();
}

void HsCloudCollectionBridge::reviceDataAndPubCB()
{

//    redisCon->setpublishForceData();
//    redisCon->publish()
}
