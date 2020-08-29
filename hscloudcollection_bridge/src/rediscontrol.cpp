#include "rediscontrol.h"
using namespace rapidjson;

redisControl::redisControl()
{
    keys = vector<string>{"Fx", "Fy", "Fz", "Tx", "Ty","Tz","time"};
    isConnect = false;
}

redisControl::redisControl(const std::vector<string> &keys)
{
    copy(keys.begin(), keys.end(), this->keys.begin());
    isConnect = false;

}


redisControl::~redisControl()
{
    redisFree(redis);     // 命令执行失败，释放内存
}



int redisControl::connect(const std::string &ip, int port)
{
    struct timeval timV;
    timV.tv_sec = 2;
//    redis = redisConnectWithTimeout(ip.c_str(), port,timV);
    redis = redisConnect(ip.c_str(), port);

    if ( NULL == redis || redis->err)
    {       // redis为NULL与redis->err是两种不同的错误，若redis->err为true，可使用redis->errstr查看错误信息
        printf("%s \r\n", redis->errstr);
        printf("Connect to redisServer faile\n");
        isConnect = false;
        redisFree(redis);

        return -1;
    }
    isConnect = true;
    return 0;
}

void redisControl::setpublishForceData(std::vector<double> &data)
{
    assert(keys.size() == data.size()+1);
    data.push_back(getTime());
    string paserStr;
    transfromJson(keys, std::move(data), paserStr);
    out.push(paserStr);
}

int redisControl::publish( const string &channel)
{
//    std::string str = "publish XZJQR_DATAARR %s";
    if(out.size() ==0 ){
        printf("Execut queue size is empty \n");
        return -1;
    }
    string getQueueUnit = out.front();
    redisReply *reply = (redisReply*)redisCommand(redis,std::move(channel.c_str()), std::move(getQueueUnit.c_str()));    // 执行命令，结果强转成redisReply*类型
    if( NULL == reply)
    {
        printf("Execut command1 failure %s \n",redis->errstr);
        out.pop();
        return -1;
    }

    out.pop();
    printf("Execut command1 ok %d \n",redis->err);

    freeReplyObject(reply);
    return 0;
}

bool redisControl::setJsonkey(const std::vector<string> &keys)
{
    this->keys.clear();
    this->keys.resize(keys.size());
    try{
        copy(keys.begin(), keys.end(), this->keys.begin());
    }catch(std::exception&e)
    {
        return false;
    }
    return true;
}

int redisControl::getJsonKey() const
{
    return keys.size();
}

bool redisControl::isConnected() const
{
    return  isConnect;
}

void redisControl::disConnect()
{
    redisFree(redis);     // 命令执行失败，释放内存
    isConnect = false;
}

void redisControl::transfromJson(const std::vector<string> &keys,  std::vector<double> &&val,std::string &out)
{
    Document document; 			// Null
    document.SetObject();		//如果不使用这个方法，AddMember将会报错
    //下面会遇到多次使用allocator的情况，采用这种方式避免多次调用GetAllocator()去获取allocator
    Document::AllocatorType& allocator = document.GetAllocator();

    /*********keys - value json 格式**************/
    Value value(kObjectType);
    for(int i = 0; i< keys.size() ; i++){
        value.SetDouble(val.at(i));
        //document添加double类型数据
        document.AddMember(StringRef(keys.at(i).c_str()), value, allocator);

    }

    StringBuffer sb;
    PrettyWriter<StringBuffer> writer(sb);
    document.Accept(writer);

    // Get the ouput string data
    std::cout <<sb.GetString()<<std::endl;
    out = std::string(sb.GetString());
}

double redisControl::getTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec;
}
