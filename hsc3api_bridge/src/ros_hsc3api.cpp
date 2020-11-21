#include "ros_hsc3api.h"

Hsc3ApiRos::Hsc3ApiRos(ros::NodeHandle n)
{
    n_hsc3 = n;
    commapi = new CommApi();
    proMo = new ProxyMotion(commapi);
    proIO = new ProxyIO(commapi);
    proSys = new ProxySys(commapi);
    proVm = new ProxyVm(commapi);
    proVar = new ProxyVar(commapi);
    commapi->setAutoConn(false);//关闭自动重连功能，否则连接失败
}

Hsc3ApiRos::~Hsc3ApiRos()
{

    delete commapi;
    delete proMo;
    delete proIO;
    delete proSys;
    proIO = NULL;
    proMo = NULL;
    commapi = NULL;
    proSys = NULL;
}

int Hsc3ApiRos::start()
{
    ret = -1;

    n_hsc3.param("/hsc3api_bridge/hsc3_robotIP", robotIp_, std::string("192.168.98.2"));
    n_hsc3.param("/hsc3api_bridge/hsc3_robotPort", robotPort_, int(23234));

    ROS_INFO("robotIp_: %s",robotIp_.c_str());
    ROS_INFO("robotPort_: %d", robotPort_);

    ret = commapi->connect(robotIp_, (uint16_t)robotPort_);
    if(ret != 0){
        ROS_ERROR("connect hsr_robot faile,return value is %d !",ret);
        return -1;
    }

    start_jog = n_hsc3.advertiseService("hsc3StartJog", &Hsc3ApiRos::startJogCB, this);
    stop_jog = n_hsc3.advertiseService("hsc3StopJog", &Hsc3ApiRos::stopJogCB, this);
    set_vord = n_hsc3.advertiseService("hsc3SetVord", &Hsc3ApiRos::setVordCB, this);
    set_opmode = n_hsc3.advertiseService("hsc3SetOpMode", &Hsc3ApiRos::setOpModeCB, this);
    move_to = n_hsc3.advertiseService("hsc3MoveTo", &Hsc3ApiRos::moveToCB, this);
    set_workfarm = n_hsc3.advertiseService("hsc3SetWorkFrame", &Hsc3ApiRos::setWorkFrameCB, this);
    set_iodout = n_hsc3.advertiseService("hsc3SetIODout", &Hsc3ApiRos::setIODoutCB, this);
    getRobotConnStatus = n_hsc3.advertiseService("getRobotConnStatus", &Hsc3ApiRos::getRobotConnStatusCB, this);
    getRobotErrorFault = n_hsc3.advertiseService("getRobotErrorFaultMsg", &Hsc3ApiRos::getRobotErrorFaultCB, this);

    setStartUpProjectSer = n_hsc3.advertiseService("setStartUpProject", &Hsc3ApiRos::setStartUpProject, this);
    setStopProjectSer = n_hsc3.advertiseService("setStopProject", &Hsc3ApiRos::setStopProject, this);

    return 0;
}

bool Hsc3ApiRos::hsc3ReConnect()
{
    if(!(commapi->isConnected())){
        ret = commapi->connect(robotIp_, (uint16_t)robotPort_);
        if(ret != 0){
            ROS_ERROR("connect hsr_robot faile,return value is %d !",ret);
        }
        return ret == 0 ? true : false;
    }
    return true;
}


bool Hsc3ApiRos::startJogCB(hirop_msgs::startJog::Request &req,hirop_msgs::startJog::Response &res)
{
    int8_t axid = req.axId;
    DirectType direct;
    if(req.direc)
        direct = POSITIVE;
    else
        direct = NEGATIVE;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    ret = proMo->startJog(gpId,axid, direct);
    res.ret = ret;
    return ret == 0 ? true : false;
}


bool Hsc3ApiRos::stopJogCB(hirop_msgs::stopJog::Request &req,hirop_msgs::stopJog::Response &res)
{
    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }
    ret = proMo->stopJog(gpId);
    res.ret = ret;
    return ret == 0 ? true : false;
}

bool Hsc3ApiRos::setVordCB(hirop_msgs::setVord::Request &req,hirop_msgs::setVord::Response &res)
{
    int32_t vord = req.vord;
    ret = -1;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    ret = proMo->setJogVord(vord);
    ret = proMo->setVord(vord);
    res.ret = ret;
    return ret == 0 ? true : false;
}

bool Hsc3ApiRos::setOpModeCB(hirop_msgs::setOpMode::Request &req,hirop_msgs::setOpMode::Response &res)
{
     OpMode mode;

     if(!hsc3ReConnect()){
         res.ret = ret;
         return false;
     }

     switch (req.mode) {
     case 1:
         mode = OP_T1;
         break;
     case 2:
         mode = OP_T2;
         break;
     case 3:
         mode = OP_AUT;
         break;
     case 4:
         mode = OP_EXT;
         break;
     default:
         mode = OP_T1;
         break;
     }
     ret = proMo->setOpMode(mode);
     res.ret = ret;
     return ret == 0 ? true : false;
}

bool Hsc3ApiRos::moveToCB(hirop_msgs::moveTo::Request &req,hirop_msgs::moveTo::Response &res)
{
    GeneralPos gpos;
    bool islinear;
    int32_t conifg;
    ret = -1;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    ret = proMo->getConfig(gpId,conifg);

    gpos.ufNum = req.gpos.ufNum;
    gpos.utNum = req.gpos.utNum;
    gpos.config = conifg;
    gpos.isJoint = req.gpos.isjoint;
    islinear = req.isLinear;

    gpos.vecPos = req.gpos.verpos.data;

    ret = proMo->moveTo(gpId, gpos, islinear);
    res.ret = ret;
    return ret == 0 ? true : false;
}

bool Hsc3ApiRos::setWorkFrameCB(hirop_msgs::setWorkFrame::Request &req, hirop_msgs::setWorkFrame::Response &res)
{
    FrameType frame;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    switch (req.frame) {
    case 1:
        frame = FRAME_JOINT;
        break;
    case 2:
        frame = FRAME_WORLD;
        break;
    case 3:
        frame = FRAME_TOOL;
        break;
    case 4:
        frame = FRAME_BASE;
        break;
    default:
        frame = FRAME_JOINT;
        break;
    }
    ret = proMo->setWorkFrame(gpId, frame);
    res.ret = ret;
    return ret == 0 ? true : false;
}

bool Hsc3ApiRos::setIODoutCB(hirop_msgs::setIODout::Request &req, hirop_msgs::setIODout::Response &res)
{
    int32_t protIo = req.portIndex;
    bool vlaue = req.value;

    if(!hsc3ReConnect()){
        res.ret = ret;
        return false;
    }

    ret = proIO->setDout(protIo, vlaue);
    res.ret = ret;
    return  ret == 0 ? true : false;
}

bool Hsc3ApiRos::getRobotConnStatusCB(hirop_msgs::robotConn::Request &req, hirop_msgs::robotConn::Response &res)
{
    bool flag = false;
    if(hsc3ReConnect())
        flag = true;
    else
        flag = false;
   res.ret =flag;
   return flag;
}

/*
 *
 *
 *  ERR_LEVEL_UNKNOWN = 0,
    ERR_LEVEL_MIN = 1,
    ERR_LEVEL_INFO = 1,     ///< <em>1</em> - 信息
    ERR_LEVEL_NOTE,         ///< <em>2</em> - 提示
    ERR_LEVEL_WARNING,      ///< <em>3</em> - 警告
    ERR_LEVEL_ERROR,        ///< <em>4</em> - 错误
    ERR_LEVEL_FATAL,        ///< <em>5</em> - 严重错误
    ERR_LEVEL_MAX,
 *
 */

bool Hsc3ApiRos::getRobotErrorFaultCB(hirop_msgs::robotErrorRequest &req, hirop_msgs::robotErrorResponse &res)
{
    req;
    uint64_t errCode;
    std::string strReason, strElim;
    if(!hsc3ReConnect()){
        return false;
    }
    ErrLevel level;
//    Hsc3::Comm::HMCErrCode getMessage(ErrLevel & level, uint64_t & code, std::string & strMsg, uint32_t ulWaitTime);
      Hsc3::Comm::HMCErrCode code =proSys->getMessage(level,errCode, strReason, 1000*5);

//    Hsc3::Comm::HMCErrCode code =proSys->queryError(errCode, strReason, strElim);
    if(code >1 ){
        res.isError = false;
        return false;
    }
    else if(errCode > 1 )
    {
        res.isError = true;
        res.errorMsg = strReason;
        res.dealMsg = strElim;
    }else{
        res.isError = false;

    }
    return true;
}

bool Hsc3ApiRos::setStartUpProject(hirop_msgs::setStartUpProject::Request &req, hirop_msgs::setStartUpProject::Response &res)
{
    ROS_INFO_STREAM("setStartUpProject ...");
     if(!hsc3ReConnect()){
         res.ret = -1;
         return false;
     }
     ROS_INFO_STREAM("hsc3ReConnect ...");

     bool en;
     // anzhuang
     ret = proMo->getGpEn(0,en);
     if(ret != 0){
         res.ret = -2;
         return false;
     }

     /************************/
     stopProgram = false;
     const string path = "./script";

     // load program
     progname = req.programName;
     ret = proVm->load(path,progname);
     if(ret != 0){
         res.ret = -3;
         return false;
     }

	 sleep(1);
     // start program
     ret = proVm->start(progname);
     if(ret != 0){
         res.ret = -4;
         return false;
     }

     double wait = 0;
     ROS_INFO_STREAM("hsc3 Say GoodBye action ......");
     proVar->setR(10,0);
     while(!stopProgram)
     {
         proVar->getR(10,wait);
         if(wait == 1)
             break;
         usleep(100000);
     }
     if(stopProgram){
         proVm->stop(progname);
         res.ret = -5;
         ret = proVm->unload(progname);
         ROS_INFO_STREAM("hsc3 Say GoodBye action stop ......");

         return true;
     }

     ret = proVm->unload(progname);
     ROS_INFO_STREAM("hsc3 Say GoodBye action fisnish......");

     res.ret = 0;
     return true;
}

bool Hsc3ApiRos::setStopProject(hirop_msgs::setStopProject::Request &req, hirop_msgs::setStopProject::Response &res)
{

    stopProgram = true;
    res.ret = 0;
    return true;
}
