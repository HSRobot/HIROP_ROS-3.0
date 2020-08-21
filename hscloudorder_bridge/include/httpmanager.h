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

using namespace std;
struct HsOrderData
{
    HsOrderData() {}
    string ID;
    string pattern;
    string Customized;
    string CompanyName;
    string PersonPhone;
    long dataTime;
};

using namespace std;
/**
 * @brief The httpManager class
 */
class httpManager
{
public:
    explicit httpManager(const string & url);
    bool connect();
    bool isVail();
    bool getOrderListNonBlock();
    void getOrder();
    bool finishOrder();
    bool setOrdeingOn();
    bool setOrdeingOff();
    void waitOrder();
    void setWaitInterrupt();
private:
    bool parseResponse(std::shared_ptr<httplib::Response> &res);
    void parseJson(const std::string &parseStr);
    void parseHsOrderData(rapidjson::Document& doc);
    string url;
    std::shared_ptr<httplib::Client> cli;
    bool waitOrderFlag, waitOrderInterruptFlag;
    std::shared_ptr<HsOrderData> hsOrderData;
};

#endif // HTTPMANAGER_H
