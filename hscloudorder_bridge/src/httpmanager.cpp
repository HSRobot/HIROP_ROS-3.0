#include "httpmanager.h"
//#define HS_GETORDERLIST_CMD "http://202.105.25.206:8082/PageCount/GetOrderList"
#define HS_GETORDERLIST_CMD "/PageCount/GetOrderList"
#define HS_SETORDERON_CMD "/PageCount/SetOrdeingOn"
#define HS_SETORDEROFF_CMD "/PageCount/SetOrdeingOff"
#define HS_FINISHORDER_CMD "/PageCount/FinishOrder?id="
//http://202.105.25.206:8082/PageCount/SetOrdeingOn
//http://202.105.25.206:8082/PageCount/SetOrdeingOff
#include <future>
using namespace rapidjson;

httpManager::httpManager(const string& url):waitOrderFlag(false),waitOrderInterruptFlag(false)
{
    this->url =  url;
    cli = nullptr;
}

bool httpManager::connect()
{

    cli = std::make_shared<httplib::Client>(url.c_str());
    if(cli == nullptr)
    {
        return false;
    }

    std::cout << "conneted to server "<<std::endl;
    return cli->is_valid();
}

bool httpManager::isVail()
{
    bool ret;
    if(cli != nullptr)
         ret  = cli->is_valid();
    else{
        std::cout <<  "httpManager object is nulllptr "<<std::endl;
        ret = false;
    }
    return ret;
}

bool httpManager::getOrderListNonBlock()
{
    std::shared_ptr<httplib::Response> res = cli->Get(HS_GETORDERLIST_CMD, [&](const char *data, size_t data_length) {
        string param = string(data, data_length);
        if(data_length < 3)
            return false;

        replaceAWithX(param, "]","");
        replaceAWithX(param, "[","");

        return parseJson(param);
      });

    return parseResponse(res);
}

std::shared_ptr<HsOrderData> &httpManager::getOrderDec()
{
    return orderQueue.front();
}

bool httpManager::finishOrder(int ID)
{
    std::shared_ptr<HsOrderData> data = orderQueue.front();
    if(data->ID != ID){
        std::cout << "finish ID ERROR  :"<<data->ID<<std::endl;
        return false;
    }
    string cmd = string(HS_FINISHORDER_CMD).append(std::to_string(ID));
//    cmd;
    std::shared_ptr<httplib::Response> res = cli->Get(cmd.c_str(), [&](const char *data, size_t data_length) {
        if("OK" ==  string(data, data_length)){
            orderQueue.pop();
            return true;
        }else
            return false;
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

bool httpManager::waitOrder()
{
    bool getOrder = false;
    if(!waitOrderFlag){
        waitOrderFlag = true;
        std::future<void> b1 = std::async([&]{
            while(waitOrderFlag){
                bool ret = getOrderListNonBlock();
                if(ret){
                    getOrder = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });
        b1.get();
        waitOrderFlag = false;
        return getOrder;
    }
    return false;
}

void httpManager::setWaitInterrupt()
{
    waitOrderFlag = false;
}

void httpManager::disConnnect()
{
    if(cli == nullptr)
        return ;
    cli = nullptr;

}

bool httpManager::parseResponse(std::shared_ptr<httplib::Response> &res)
{
    if(res == nullptr)
        return false;
    if( res->status == 200)
        return true;

    std::cout << "#ERROR http code is res->status : "<<res->status<<std::endl;
    return false;
}



bool httpManager::parseJson(const string &parseStr)
{
    std::vector<std::string> splitVec = split(parseStr, string("}"));
    string temp = splitVec.at(0);
    rapidjson::Document document;
    document.Parse(temp.c_str());
    std::cout << temp<<std::endl;
    if (document.ParseInsitu(const_cast<char*>(temp.c_str())).HasParseError()){
        return false;
    }
    assert(document.IsObject());
    std::shared_ptr<HsOrderData> hsOrderData = std::make_shared<HsOrderData>();
    parseHsOrderData(hsOrderData.get(), document);
    if( orderQueue.size() >0 &&hsOrderData->ID != orderQueue.front()->ID)
        orderQueue.push(hsOrderData);
    else if( orderQueue.size() ==0 )
    {
        orderQueue.push(hsOrderData);
    }
    return true;
}

void httpManager::parseHsOrderData(HsOrderData *orderData, Document &doc)
{
    assert(doc.IsObject());

    orderData->ID = doc["ID"].GetInt();
    orderData->CompanyName = doc["CompanyName"].GetString();
    orderData->PersonName = doc["PersonName"].GetString();
    orderData->VerificationCode = doc["VerificationCode"].GetString();
    orderData->CreateTime = doc["CreateTime"].GetString();
}

void httpManager::replaceAWithX(string &str, const char *find, const char *replace)
{
    int pos;
    pos = str.find(find);
    while(pos != -1){
        // str.length()求字符的长度，注意str必须是string类型
        str.replace(pos,string(find).length(),replace);
        pos = str.find(find);
    }
}

std::vector<string> httpManager::split(string str, string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    int size = str.size();
    for (int i = 0; i < size; i++)
    {
        //第一个参数是 寻找的字符，第二个是从什么地址开始搜索
        pos = str.find(pattern, i);
        if (pos < size)
        {
            //substr 第一个参数是 起始地址 第二个是 偏移地址长度
            std::string s = str.substr(i, pos -i+1);
            int posFirst = s.find_first_of(",");
            if(posFirst < 1)
                s.erase(s.begin(),s.begin()+1);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}
