#ifndef HTTPMANAGER_H
#define HTTPMANAGER_H
/**
  @author KONGDELIANG
  @date 2020-08-15
 */
#include <httplib.h>
#include <memory>
#include <string>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <queue>
using namespace std;
struct HsOrderData
{
    HsOrderData() {}
    int ID;
    int pattern;
    string Customized;
    string CompanyName;
    string PersonName;
    string VerificationCode;
    string CreateTime;
};

struct HsOrderDataObject
{
    HsOrderDataObject() {}
    int ID;
    int pattern;
    string Customized;
    string CompanyName;
    int sendState;
    string CreateTime;
};

using namespace std;
/**
 * @brief The httpManager class
 */
class httpManager
{
public:
    explicit httpManager(const string & url);
    /**
     * @brief connect 连接
     * @return
     */
    bool connect();

    /**
     * @brief isVail 查看是否有效
     * @return
     */
    bool isVail();

    /**
     * @brief getOrderListNonBlock 获取云订单 非阻塞
     * @return
     */
    bool getOrderListNonBlock();


    std::shared_ptr<HsOrderData> &getOrderDec();

    /**
     * @brief waitOrder 获取云订单 阻塞
     */
    bool waitOrder();

    /**
     * @brief finishOrder 完成云下单
     * @return
     */
    bool finishOrder(int ID);

    /**
     * @brief setOrdeingOn 开始云下单
     * @return
     */
    bool setOrdeingOn();

    /**
     * @brief setOrdeingOff 停止云下单
     * @return
     */
    bool setOrdeingOff();

    /**
     * @brief setWaitInterrupt 中断云下单阻塞
     */
    void setWaitInterrupt();

    /**
     * @brief disConnnect
     */
    void disConnnect();

private:
    bool parseResponse(std::shared_ptr<httplib::Response> &res);
    /**
     * @brief parseJson 主要业务的JSON格式解析
     * @param parseStr
     */
    bool parseJson(const std::string &parseStr);
    void parseHsOrderData(HsOrderData *orderData, rapidjson::Document& doc);
    void replaceAWithX(string& str, const char* find, const char * replace);

    std::vector<std::string> split(string str, string pattern);
    string url;
    std::shared_ptr<httplib::Client> cli;
    bool waitOrderFlag, waitOrderInterruptFlag;
//    std::shared_ptr<HsOrderData> hsOrderData;

    std::queue<std::shared_ptr<HsOrderData>> orderQueue;
};

#endif // HTTPMANAGER_H
