#include "rediscontrol.h"
using namespace rapidjson;

redisControl::redisControl()
{
    keys = vector<string>{"Fx", "Fy", "Fz", "Tx", "Ty","Tz","time"};
}

redisControl::redisControl(const std::vector<string> &keys)
{
    copy(keys.begin(), keys.end(), this->keys.begin());
}


redisControl::~redisControl()
{
    redisFree(redis);     // 命令执行失败，释放内存
}



int redisControl::connect(const std::string &ip, int port)
{
    redis = redisConnect(ip.c_str(), port);

    if ( NULL == redis || redis->err)
    {       // redis为NULL与redis->err是两种不同的错误，若redis->err为true，可使用redis->errstr查看错误信息
        redisFree(redis);
        printf("Connect to redisServer faile\n");
        return -1;
    }
    return 0;
}

void redisControl::setpublishForceData(std::vector<double> &data)
{
    assert(keys.size() == data.size()+1);
    data.push_back(getTime());
    string paserStr;
    transfromJson(keys, data, paserStr);
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
        printf("Execut command1 failure\n");
        return -1;
    }

    out.pop();

    freeReplyObject(reply);
    return 0;
}

void redisControl::setJsonkey(const std::vector<string> &keys)
{
    this->keys.clear();
    this->keys.resize(keys.size());
    copy(keys.begin(), keys.end(), this->keys.begin());
}

int redisControl::getJsonKey() const
{
    return keys.size();
}

void redisControl::transfromJson(const std::vector<string> &keys, const std::vector<double> &val,std::string &out)
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
