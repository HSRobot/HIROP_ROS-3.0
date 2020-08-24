#pragma once
#include <ros/ros.h>
#include <httpmanager.h>
#include <std_srvs/SetBool.h>
#include <hirop_msgs/cancelCloudOrder.h>
#include <hirop_msgs/getCloudOrder.h>
#include <hirop_msgs/finishCloudOrder.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
class HsCloudOrderBridge
{
public:
    HsCloudOrderBridge(ros::NodeHandle & nh);

    void start();
private:
    /**
     * @brief connectCB 连接云服务器
     * @param req
     * @param res
     * @return
     */
    bool connectCB(std_srvs::TriggerRequest & req, std_srvs::TriggerResponse& res);

    /**
     * @brief disConnectCB 断开云服务器
     * @param req
     * @param res
     * @return
     */
    bool disConnectCB(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
//    void closeCB(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

    /**
     * @brief getOrderBlock 获取云订单
     * @param req
     * @param res
     * @return
     */
    bool getOrderBlock(hirop_msgs::getCloudOrderRequest &req, hirop_msgs::getCloudOrderResponse &res);

    /**
     * @brief getOrderInterrupt 获取云订单取消
     * @param req
     * @param res
     * @return
     */
    bool getOrderInterrupt(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

    /**
     * @brief finishOrder 完成云订单
     * @param req
     * @param res
     * @return
     */
    bool finishOrder(hirop_msgs::finishCloudOrderRequest &req, hirop_msgs::finishCloudOrderResponse &res);

    /**
     * @brief setOrderEnableCB
     * @param req
     * @param res
     * @return
     */
    bool setOrderEnableCB(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

private:
    void debug(const char * str);
private:
    ros::NodeHandle nh;
    std::shared_ptr<httpManager> httpManagerPtr;
    std::string orderUrl;
    ros::ServiceServer connectSer;
    ros::ServiceServer disConnectSer;
    ros::ServiceServer getOrderSer;
    ros::ServiceServer getOrderCancelSer;
    ros::ServiceServer finishOrderSer;
    ros::ServiceServer setOrderEnableSer;

};
