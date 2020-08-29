#include <ros/ros.h>
#include <hscloudcollection_bridge_node.h>


HsCloudCollectionBridge::HsCloudCollectionBridge(ros::NodeHandle &nh):ip("127.0.0.1"),port(6379)
{
    this->nh = nh;
    redisCon = boost::make_shared<redisControl>();
    connectCloudSer = nh.advertiseService("connectCloudSer", &HsCloudCollectionBridge::connectCloudSrvCB,this);
    disConnectCloudSer = nh.advertiseService("disConnectCloudSer", &HsCloudCollectionBridge::disConnectCloudCB,this);

    setKeyCloudSer = nh.advertiseService("setKeyCloudSer", &HsCloudCollectionBridge::setJsonKeySrvCB,this);
    recevForceData.clear();
    receFalg = false;
}

void HsCloudCollectionBridge::start()
{
    std::vector<double> data;

    while(ros::ok())
    {
        if(redisCon->isConnected())
        {
              if(force_sub == nullptr)
              {
                  force_sub = nh.subscribe("/daq_data", 1, &HsCloudCollectionBridge::reviceDataAndPubCB, this,\
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
              }

              if(receFalg == true);
              {
                  data.clear();
                  data.resize(6);
                  copy(recevForceData.begin(), recevForceData.begin()+ recevForceData.size(), data.begin());
//                  data = recevForceData;
                  receFalg = false;
                  redisCon->setpublishForceData(data);
                  recevForceData.clear();
                  redisCon->publish();
                  ROS_INFO_STREAM("redisCon data send sucessfully ");

                  force_sub = nh.subscribe("/daq_data", 1, &HsCloudCollectionBridge::reviceDataAndPubCB, this,\
                                         ros::TransportHints().unreliable().maxDatagramSize(1000));
              }

        }
        ros::Duration(1).sleep(); // sleep for half a second
    }
}

bool HsCloudCollectionBridge::connectCloudSrvCB(hirop_msgs::connectCloudRequest &req, hirop_msgs::connectCloudResponse&res)
{
    string ip = req.ip;
    int ret = 0;
    if (ip.size() < 2)
        ret = redisCon->connect();
    else
        ret = redisCon->connect(ip, port);

    res.ret = ret == 0?true:false;
    if(res.ret)
        ROS_ERROR_STREAM("server i connected! ");
    else
        ROS_ERROR_STREAM("server is not  connected! ");

    return true;
}

bool HsCloudCollectionBridge::setJsonKeySrvCB(hirop_msgs::setCloudKeyRequest &req, hirop_msgs::setCloudKeyResponse &res)
{
    if(!redisCon->isConnected())
    {
        ROS_ERROR_STREAM("server is not connected! ");
        return false;
    }
    std::vector<string> keys = req.keys;
    res.ret = redisCon->setJsonkey(keys);
    return true;
}

void HsCloudCollectionBridge::reviceDataAndPubCB(const geometry_msgs::Wrench::ConstPtr &msg)
{
    if(!redisCon->isConnected())
    {
        return ;
    }
    recevForceData = {msg->force.x,msg->force.y,msg->force.z,\
                                msg->torque.x,msg->torque.y,msg->torque.z };

    receFalg = true;
    force_sub.shutdown();

}

bool HsCloudCollectionBridge::disConnectCloudCB(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    redisCon->disConnect();
    return true;
}
