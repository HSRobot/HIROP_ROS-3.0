#include <ros/ros.h>
#include <rediscontrol.h>
#include <boost/shared_ptr.hpp>
#include <hirop_msgs/connectCloud.h>
#include <hirop_msgs/setCloudKey.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>
#include <boost/atomic.hpp>

class HsCloudCollectionBridge
{
public:
    HsCloudCollectionBridge(ros::NodeHandle &nh);
    void start();
private:
    /**
     * @brief connectCloudSrvCB 连接服务
     * @param req
     * @param res
     * @return
     */
    bool connectCloudSrvCB(hirop_msgs::connectCloudRequest &req, hirop_msgs::connectCloudResponse &res);

    /**
     * @brief setJsonKeySrvCB
     * @param req
     * @param res
     * @return
     */
    bool setJsonKeySrvCB(hirop_msgs::setCloudKeyRequest& req, hirop_msgs::setCloudKeyResponse& res);

    /**
     * @brief reviceDataAndPubCB
     * @param msg
     */
    void reviceDataAndPubCB(const geometry_msgs::Wrench::ConstPtr &msg);

    /**
     * @brief disConnectCloudCB 断开服务
     * @param req
     * @param res
     * @return
     */
    bool disConnectCloudCB(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse& res);
private:
    boost::shared_ptr<redisControl> redisCon;
    string ip;
    int port;
    ros::NodeHandle nh;
    ros::ServiceServer setKeyCloudSer;
    ros::ServiceServer connectCloudSer,disConnectCloudSer;
    ros::Subscriber   force_sub;
    std::vector<double> recevForceData;
    boost::atomic<bool> receFalg;
};
