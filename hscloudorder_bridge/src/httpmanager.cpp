#include "httpmanager.h"
//#define HS_GETORDERLIST_CMD "http://202.105.25.206:8082/PageCount/GetOrderList"
#define HS_GETORDERLIST_CMD "/PageCount/GetOrderList"
#define HS_SETORDERON_CMD "/PageCount/SetOrdeingOn"
#define HS_SETORDEROFF_CMD "/PageCount/SetOrdeingOff"
#define HS_FINISHORDER_CMD "/PageCount/FinishOrder"

//http://202.105.25.206:8082/PageCount/SetOrdeingOn
//http://202.105.25.206:8082/PageCount/SetOrdeingOff
#include <future>
using namespace rapidjson;

httpManager::httpManager(const string& url):waitOrderFlag(false),waitOrderInterruptFlag(false)
{
    this->url =  url;
}

bool httpManager::connect()
{

    cli = std::make_shared<httplib::Client>(url.c_str());
    if(cli == nullptr)
    {
        return false;
    }
    std::cout << "conneted to server "<<std::endl;
    return true;
}

bool httpManager::isVail()
{
     return cli->is_valid();
}

bool httpManager::getOrderListNonBlock()
{
    if(waitOrderFlag)
        return false;
    std::shared_ptr<httplib::Response> res = cli->Get(HS_GETORDERLIST_CMD, [&](const char *data, size_t data_length) {
//        std::cout << <<std::endl;
        parseJson(string(data, data_length));
        return true;
      });

    return parseResponse(res);
}

void httpManager::getOrder()
{

}

bool httpManager::finishOrder()
{
    std::shared_ptr<httplib::Response> res = cli->Get(HS_FINISHORDER_CMD, [&](const char *data, size_t data_length) {
        std::cout << string(data, data_length)<<std::endl;
        return true;
      });

    return parseResponse(res);
}

bool httpManager::setOrdeingOn()
{
    std::shared_ptr<httplib::Response> res = cli->Get(HS_SETORDERON_CMD, [&](const char *data, size_t data_length) {
        std::cout << string(data, data_length)<<std::endl;
        return true;
      });

    return parseResponse(res);
}

bool httpManager::setOrdeingOff()
{
    std::shared_ptr<httplib::Response> res = cli->Get(HS_SETORDEROFF_CMD, [&](const char *data, size_t data_length) {
        std::cout << string(data, data_length)<<std::endl;
        return true;
    });

    return parseResponse(res);
}

void httpManager::waitOrder()
{
    if(!waitOrderFlag){
        waitOrderFlag = true;
        std::future<bool> b1 = std::async([&]{
            while(waitOrderFlag){
                bool ret = getOrderListNonBlock();
                if(ret)
                    return true;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });
        b1.get();
    }
}

void httpManager::setWaitInterrupt()
{
    waitOrderFlag = false;
}

bool httpManager::parseResponse(std::shared_ptr<httplib::Response> &res)
{
    if( res->status == 500)
        return true;
    return false;
}

void httpManager::parseJson(const string &parseStr)
{
    hsOrderData = std::make_shared<HsOrderData>();
    rapidjson::Document document;
    document.Parse(parseStr.c_str());

    parseHsOrderData(document);


}

void httpManager::parseHsOrderData(Document &doc)
{
    rapidjson::Value::ConstMemberIterator iter = doc.FindMember("ID");
    if(iter != doc.MemberEnd()){
        cout << "ID : " << iter->value.GetString() << endl;
    }

//    rapidjson::Value::ConstMemberIterator iter = doc.FindMember("Pattern");
//    if(iter != doc.MemberEnd()){
//        cout << "name : " << iter->value.GetString() << endl;
//    }

//    rapidjson::Value::ConstMemberIterator iter = doc.FindMember("ID");
//    if(iter != doc.MemberEnd()){
//        cout << "name : " << iter->value.GetString() << endl;
//    }
}
