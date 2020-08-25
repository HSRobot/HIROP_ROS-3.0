
#include <string>
#include <iostream>
#include "rediscontrol.h"
#include <thread>
using namespace rapidjson;

int main()
{
    redisControl red;
    assert(0 == red.connect("127.0.0.1", 6379));

//    for(int i = 0 ; i < 55;  i++){
    for(;;){
        vector<double> temp{1, 2,3,4,5,6};
        red.setpublishForceData(temp);
        if(0!= red.publish())
        {
            perror("publish error");
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "publish ok "<<std::endl;
    }
    return 0;
}
