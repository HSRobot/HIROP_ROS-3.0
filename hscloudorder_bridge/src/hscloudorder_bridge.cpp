#include <hscloudorder_bridge.h>

HsCloudOrderBridge::HsCloudOrderBridge(ros::NodeHandle &nh):nh(nh)
{
    nh.param("orderUrl",orderUrl, std::string("http://202.105.25.206:8082"));
    httpManagerPtr = std::make_shared<httpManager>(orderUrl);
    connectSer = nh.advertiseService("connectCloud", &HsCloudOrderBridge::connectCB, this);
    disConnectSer = nh.advertiseService("disConnectCloud", &HsCloudOrderBridge::disConnectCB, this);
    getOrderSer = nh.advertiseService("getOrderSer", &HsCloudOrderBridge::getOrderBlock, this);
    getOrderCancelSer = nh.advertiseService("getOrderCancelSer", &HsCloudOrderBridge::getOrderInterrupt, this);
    finishOrderSer = nh.advertiseService("finishOrderSer", &HsCloudOrderBridge::finishOrder, this);
    setOrderEnableSer = nh.advertiseService("setOrderEnableSer", &HsCloudOrderBridge::setOrderEnableCB, this);
}

void HsCloudOrderBridge::start()
{
    debug("HsCloudOrderBridge start.....");
    ros::MultiThreadedSpinner sp(1);
    sp.spin();
}

bool HsCloudOrderBridge::connectCB(std_srvs::TriggerRequest & req, std_srvs::TriggerResponse& res)
{
    debug("Cloud is connected .....");

    res.success = 1;
    return httpManagerPtr->connect();
}

bool HsCloudOrderBridge::disConnectCB(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    httpManagerPtr->disConnnect();
    debug("Cloud disconnect .....");

    return true;
}

bool HsCloudOrderBridge::getOrderBlock(hirop_msgs::getCloudOrderRequest &req, hirop_msgs::getCloudOrderResponse &res)
{
    if(!httpManagerPtr->isVail())
        return false;

    if(req.bloclEnable == 0)
    {
        if(!httpManagerPtr->getOrderListNonBlock())
        {
            debug("Please Notice getOrderList NonBlock is non !");
            return false;
        }

    }else if(req.bloclEnable==1 && !httpManagerPtr->waitOrder()){
        debug("Please Notice:  http clould BLOCK order is interrupt !");
        return false;
    }
    std::shared_ptr<HsOrderData> data = httpManagerPtr->getOrderDec();
    res.desc.ID = data->ID;
    res.desc.companyName = data->CompanyName;
    res.desc.phoneName = data->PersonName;
    return true;
}

bool HsCloudOrderBridge::getOrderInterrupt(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    if(!httpManagerPtr->isVail())
        return false;
    httpManagerPtr->setWaitInterrupt();
    return true;
}

bool HsCloudOrderBridge::finishOrder(hirop_msgs::finishCloudOrderRequest &req, hirop_msgs::finishCloudOrderResponse &res)
{
    if(!httpManagerPtr->isVail())
        return false;
    return httpManagerPtr->finishOrder(req.ID);
}

bool HsCloudOrderBridge::setOrderEnableCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
    if( req.data == 1)
    {
        httpManagerPtr->setOrdeingOn();
    }else if (req.data == 0){
        httpManagerPtr->setOrdeingOff();
    }

    return true;
}

void HsCloudOrderBridge::debug(const char *str)
{
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //转为字符串
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
#ifdef NODEBUG
#else
    std::cout<<str_time <<"  " << str<<std::endl;
#endif
}
