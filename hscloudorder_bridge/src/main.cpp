#include <iostream>
#include "utilty.h"
#include <rapidjson/rapidjson.h>
#include <httplib.h>
#include <httpmanager.h>
using namespace std;
//http://202.105.25.206:8082/PageCount/FinishOrder?id=3889
//http://202.105.25.206:8082/PageCount/GetOrderList
//http://202.105.25.206:8082/PageCount/SetOrdeingOn
//http://202.105.25.206:8082/PageCount/SetOrdeingOff

int main()
{

    httpManager manager("http://202.105.25.206:8082");
    manager.connect();
    manager.getOrderListNonBlock();
//    manager.waitOrder();


//    httplib::Client cli("http://202.105.25.206:8082");
//    std::shared_ptr<Response> res = cli.Get("/PageCount/GetOrderList", [&](const char *data, size_t data_length) {
//        std::cout << string(data, data_length)<<std::endl;
//        return true;
//      });
//    if (res) {
//        cout << res->status << endl;
//        cout << res->get_header_value("Customized") << endl;
//        cout << res->body << endl;
//    }
//    cout << "Hello World!" << endl;
//    while(1);
    return 0;
}
