#ifndef REDISCONTROL_H
#define REDISCONTROL_H
#include <iostream>
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h" // for stringify JSON
#include <vector>
#include <hiredis/hiredis.h>
#include <string>
#include <queue>
using namespace std;
class redisControl
{
public:
    redisControl();
    ~redisControl();
    redisControl(const std::vector<string> &keys);

    /**
     * @brief connect 连接redis 服务器
     * @param ip      ip地址
     * @param port    端口号
     * @return
     */
    int connect(const std::string &ip, int port);

    /**
     * @brief setpublishForceData
     * @param data
     */
    void setpublishForceData( std::vector<double> &data);

    /**
     * @brief publish 服务器上传
     * @param channel 特定的管道
     * @return 返回的参数
     */
    int publish(const std::string &channel = "publish XZJQR_DATAARR %s");

private:
    void transfromJson(const std::vector<string> &keys, const std::vector<double> &val, string &out);
    double getTime();
    redisContext* redis;
    std::vector<std::string> keys;
    std::vector<double> val;
    std::queue<std::string> out;
};

#endif // REDISCONTROL_H
