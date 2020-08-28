#include <ros/ros.h>
#include <rediscontrol.h>
#include <boost/shared_ptr.hpp>
class HsCloudCollectionBridge
{
public:
    HsCloudCollectionBridge(ros::NodeHandle &nh);
    void start();
private:
    void connectCloudSrvCB();
    void setJsonKeySrvCB();
    void reviceDataAndPubCB();
private:
    boost::shared_ptr<redisControl> redisCon;
    string ip;
    int port;
    ros::NodeHandle nh;
};
