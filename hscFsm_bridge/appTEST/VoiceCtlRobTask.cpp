#include "VoiceCtlRobTask.h"

using namespace HsFsm;
typedef std::pair<string, int> elment;

VoiceCtlRobTask::VoiceCtlRobTask(const string &taskName)
{
    this->taskName = taskName;
    isStop= false;
    isErr= false;
    VCRRF=new VoiceCtlRobRosFunc(getRosHandler());

}

void VoiceCtlRobTask::init()
{

    // 自动转为 init 状态
    // 用户自定义添加内容
    cout<<" 自动转为 init 状态"<<endl;
//    system("rosservice call /stop_motion &");
}

void VoiceCtlRobTask::quit()
{
    cout<<"没用的quit"<<endl;
    //会自动转状态为 quit
    // 用户自定义添加内容
}

/**
 * @brief shakeHandTask::registerTaskList 用户重点关心
 * @return
 */
//
bool VoiceCtlRobTask::registerTaskList()
{
    auto func_init_initing = std::bind(&VoiceCtlRobTask::init_initing, this, placeholders::_1);
    auto func_init_quiting = std::bind(&VoiceCtlRobTask::init_quiting, this, placeholders::_1);
    auto func_prepare_initing = std::bind(&VoiceCtlRobTask::prepare_initing, this, placeholders::_1);
    auto func_prepare_quiting = std::bind(&VoiceCtlRobTask::prepare_quiting, this, placeholders::_1);
    auto func_Detection_initing = std::bind(&VoiceCtlRobTask::Detection_initing, this, placeholders::_1);
    auto func_Detection_quiting = std::bind(&VoiceCtlRobTask::Detection_quiting, this, placeholders::_1);
    auto func_Shakehand_initing = std::bind(&VoiceCtlRobTask::Shakehand_initing, this, placeholders::_1);
    auto func_Shakehand_quiting = std::bind(&VoiceCtlRobTask::Shakehand_quiting, this, placeholders::_1);
    auto func_Wave_initing = std::bind(&VoiceCtlRobTask::Wave_initing, this, placeholders::_1);
    auto func_Wave_quiting = std::bind(&VoiceCtlRobTask::Wave_quiting, this, placeholders::_1);
    auto func_DetectToy_initing = std::bind(&VoiceCtlRobTask::DetectToy_initing, this, placeholders::_1);
    auto func_DetectToy_quiting = std::bind(&VoiceCtlRobTask::DetectToy_quiting, this, placeholders::_1);
    auto func_grabToy_initing = std::bind(&VoiceCtlRobTask::grabToy_initing, this, placeholders::_1);
    auto func_grabToy_quiting = std::bind(&VoiceCtlRobTask::grabToy_quiting, this, placeholders::_1);
    auto func_dealErr_initing= std::bind(&VoiceCtlRobTask::dealErr_initing, this, placeholders::_1);
    auto func_dealErr_quiting= std::bind(&VoiceCtlRobTask::dealErr_quiting, this, placeholders::_1);
    auto func_exit_initing = std::bind(&VoiceCtlRobTask::exit_initing, this, placeholders::_1);
    auto func_exit_quiting = std::bind(&VoiceCtlRobTask::exit_quiting, this, placeholders::_1);

    auto func_transPrepare2detection = std::bind(&VoiceCtlRobTask::transPrepare2detection, this, placeholders::_1);
    auto func_transPrepare2Exit = std::bind(&VoiceCtlRobTask::transPrepare2Exit, this, placeholders::_1);
    auto func_transDetection2exit = std::bind(&VoiceCtlRobTask::transDetection2exit, this, placeholders::_1);
    auto func_transDetection2dealErr = std::bind(&VoiceCtlRobTask::transDetection2dealErr, this, placeholders::_1);
    auto func_transShakehand2exit = std::bind(&VoiceCtlRobTask::transShakehand2exit, this, placeholders::_1);
    auto func_transShakehand2dealErr = std::bind(&VoiceCtlRobTask::transShakehand2dealErr, this, placeholders::_1);
    auto func_transWave2exit = std::bind(&VoiceCtlRobTask::transWave2exit, this, placeholders::_1);
    auto func_transWave2dealErr = std::bind(&VoiceCtlRobTask::transWave2dealErr, this, placeholders::_1);
    auto func_transDetectToy2exit = std::bind(&VoiceCtlRobTask::transDetectToy2exit, this, placeholders::_1);
    auto func_transDetectToy2dealErr = std::bind(&VoiceCtlRobTask::transDetectToy2dealErr, this, placeholders::_1);
    auto func_transgrabToy2exit = std::bind(&VoiceCtlRobTask::transgrabToy2exit, this, placeholders::_1);
    auto func_transgrabToy2dealErr = std::bind(&VoiceCtlRobTask::transgrabToy2dealErr, this, placeholders::_1);
    auto func_transexit2init = std::bind(&VoiceCtlRobTask::transexit2init, this, placeholders::_1);

    try{
        registerTask("init","initing", func_init_initing);//to---prepare
        registerTask("init","quiting", func_init_quiting);
        registerTask("test", "initing",[&](const std::vector<std::string> &args){
            VCRRF->RobGoHome();
//            VCRRF->detectPose();
//            VCRRF->RobGoHome();
            VCRRF->robGotoShakeHandPose();
            VCRRF->RobGoHome();
//            VCRRF->wave();
            return;
            string name ="toy1";
            VCRRF->detect(name);
            sleep(5);
            geometry_msgs::PoseStamped a =VCRRF->getStateMonitor().object_pose;
            if(a.header.frame_id != "world"){
                cout<<"转换失败"<<endl;
                return;
            }
            if(VCRRF->pick(a)!=0){
                cout<<"抓取失败"<<endl;
                return;
            }

            geometry_msgs::PoseStamped b = VCRRF->placeObjPose[0];
            if(VCRRF->place(b)!=0){
                cout<<"放置失败"<<endl;
                return;
            }
            VCRRF->RobGoHome();
//            cout << "i: " << i << endl;
//        }
        });
        registerTask("prepare","initing", func_prepare_initing);
        registerTask("prepare","start", func_transPrepare2detection);//to---detection
        registerTask("prepare","quiting", func_prepare_quiting);//to---detection
        registerTask("prepare","toExit", func_transPrepare2Exit);//to---exit
        registerTask("detection","initing",func_Detection_initing);//to---shakehand/detectToy/wave
        registerTask("detection","quiting",func_Detection_quiting);

        registerTask("shakehand","initing", func_Shakehand_initing);//to---detection
        registerTask("shakehand","quiting", func_Shakehand_quiting);
        registerTask("wave","initing", func_Wave_initing);//to---detection
        registerTask("wave","quiting", func_Wave_quiting);
        registerTask("detectToy","initing",func_DetectToy_initing);//to---grabToy
        registerTask("detectToy","quiting",func_DetectToy_quiting);

        registerTask("grabToy","initing",func_grabToy_initing);//to---detection
        registerTask("grabToy","quiting",func_grabToy_quiting);
        registerTask("exit","initing", func_exit_initing);
        registerTask("exit","quiting", func_exit_quiting);//to---init
        registerTask("exit","start", func_transexit2init);//to---init
        registerTask("dealErr","initing", func_dealErr_initing);//to---exit
        registerTask("dealErr","quiting", func_dealErr_quiting);

        //所有状态故障处理
        registerTask("detection","toDealErr", func_transDetection2dealErr);//to---DealErr
        registerTask("shakehand","toDealErr", func_transShakehand2dealErr);//to---DealErr
        registerTask("wave","toDealErr", func_transWave2dealErr);          //to---DealErr
        registerTask("detectToy","toDealErr", func_transDetectToy2dealErr);//to---DealErr
        registerTask("grabToy","toDealErr", func_transgrabToy2dealErr);    //to---DealErr
        //所有状态退出处理
        registerTask("detection","toExit", func_transDetection2exit);//to---exit
        registerTask("shakehand","toExit",func_transShakehand2exit); //to---exit
        registerTask("wave","toExit", func_transWave2exit);          //to---exit
        registerTask("detectToy","toExit", func_transDetectToy2exit);//to---exit
        registerTask("grabToy","toExit", func_transgrabToy2exit);    //to---exit

    }catch(std::exception &e)
    {
        std::cout  << e.what()<<std::endl;
        assert(-1);
        return false;
    }

    return true;
}



/******************************状态行为函数**************************************************/
//init状态
void VoiceCtlRobTask::init_initing(const std::vector<std::string> &args) {
    sleep(1);
    isRun_init=true;
    typeCode = 0;
    taskRunStatus = true;
    publishStateMsg(true,"initing","-----init_initing------");
    //初始化ros信号变量
    VCRRF->initStateMonitor();
//    setTaskState("test");
    setTaskState("prepare");
    isRun_init= false;
}

void VoiceCtlRobTask::init_quiting(const std::vector<std::string> &args) {
    while (isRun_init){
        usleep(10);
    }
    publishStateMsg(true,"quiting","-----init_quiting------");
    cout<<"init完成"<<endl;
}
//prepare状态 等待信号触发完成启动
void VoiceCtlRobTask::prepare_initing(const std::vector<std::string> &args) {
    publishStateMsg(true,"initing","-----prepare_initing------");
    cout<<"进入准备状态"<<endl;
}

void VoiceCtlRobTask::prepare_quiting(const std::vector<std::string> &args){
    publishStateMsg(true,"quiting","-----prepare_quiting------");
}


void VoiceCtlRobTask::transPrepare2detection(const vector<std::string> &args) {
    publishStateMsg(true,"start","-----prepare_start------");
    //打开声音
    cout<<"打开声音"<<endl;
    VCRRF->VoiceDetect_Switch(true);
    //打开行人检测
    cout<<"打开行人检测 "<<endl;
    VCRRF->PersonDetect_Switch(true);
    VCRRF->RobGoHome();
    setTaskState("detection");
}

void VoiceCtlRobTask::transPrepare2Exit(const std::vector<std::string> &args) {
    publishStateMsg(true,"toExit","-----prepare_toExit------");
    notityRecall();
    setTaskState("exit");
}
//检测状态
void VoiceCtlRobTask::Detection_initing(const std::vector<std::string> &args) {
    publishStateMsg(true,"initing","-----detection_initing------");
    isRun_detection=true;
    //ros信号变量初始化
    VCRRF->initStateMonitor();
    int count_time=0;
    //循环检测
    while ((!isStop)&&(!isErr)&&(ros::ok()))
    {
        sleep(1);
    //
        if(VCRRF->getStateMonitor().hasPeople == 0){
            VCRRF->set_hasVoiceOder(-1);
            cout<<"没有行人"<<endl;
        }
    //0.没有行人10s&&有行人标志&&没有握手任务标志---->并挥手并回原点
        if ((VCRRF->getStateMonitor().hasPeople == 0) &&(flag_hasPeople)&&(!taskFlag_shakehand))
        {
            count_time++;
            cout<<"没有人进行，计时:"<<count_time<<endl;
            if(count_time>10)
            {
                flag_hasPeople= false;
                setTaskState("wave");
                isRun_detection= false;
                return;
            }
        }
        cout<<"detect "<<0<<endl;
    //1.没有行人10s&&有握手任务标志---->关闭阻抗并挥手并回原点
        if ((VCRRF->getStateMonitor().hasPeople == 0) &&(taskFlag_shakehand))
        {
            taskFlag_shakehand= false;
            count_time++;
            if(count_time>10)
            {
                VCRRF->closeImpedence();
                setTaskState("wave");
                isRun_detection= false;
                return;
            }
        }
        cout<<"detect "<<1<<endl;
    //2.有行人&&有握手任务标志&&握手结束信号-->关闭阻抗并回原点
        if ((VCRRF->getStateMonitor().hasPeople == 1) &&(taskFlag_shakehand)&&(VCRRF->getStateMonitor().isEnd_shakeHand))
        {
            taskFlag_shakehand= false;
            VCRRF->closeImpedence();
            //回原点
            VCRRF->RobGoHome();
            setTaskState("detection");
            isRun_detection=false;
            return;
        }
        cout<<"detect "<<2<<endl;
    //3.有行人&&有语音信号2&&没有握手任务标志--->进入握手状态
        if ((VCRRF->getStateMonitor().hasPeople == 1) &&(!taskFlag_shakehand)&&(VCRRF->getStateMonitor().voice_order==2))
        {
            taskFlag_shakehand= false;
            setTaskState("shakehand");
            isRun_detection=false;
            return;
        }
        cout<<"detect "<<3<<endl;
    //4.有行人&&有语音信号3&&没有握手任务标志-->进入抓娃娃状态
        if ((VCRRF->getStateMonitor().hasPeople == 1) &&(!taskFlag_shakehand)&&(VCRRF->getStateMonitor().voice_order==3))
        {
            taskFlag_shakehand= false;
            setTaskState("detectToy");
            isRun_detection=false;
            return;
        }
    }
    cout<<"detect "<<4<<endl;
    isRun_detection=false;
}

void VoiceCtlRobTask::Detection_quiting(const std::vector<std::string> &args) {
    publishStateMsg(true,"quiting","-----detection_quiting------");
    isStop=true;
    while (isRun_detection){
        usleep(10);
    }
    isStop= false;
    publishStateMsg(true,"quiting","-----detection_quiting------");

    cout<<"Detection_initing结束"<<endl;
}

void VoiceCtlRobTask::transDetection2exit(const std::vector<std::string> &args) {
    publishStateMsg(true,"toExit","-----detection_toExit------");
    setTaskState("exit");
    cout<<"运行exit结束"<<endl;
}

void VoiceCtlRobTask::transDetection2dealErr(const std::vector<std::string> &args) {
    publishStateMsg(true,"toDealErr","-----detection_toDealErr------");
    isErr=true;
    setTaskState("dealErr");
}

//握手状态
void VoiceCtlRobTask::Shakehand_initing(const vector<std::string> &args) {
    publishStateMsg(true,"initing","-----shakehand_initing------");
    isRun_shakehand=true;
    cout<<"握手"<<endl;
    //1.判断机器人是否上使能
    if(!VCRRF->getStateMonitor().RobEnableState){
        setTaskState("dealErr");
        cout<<"机器人没上使能,进入处理故障状态"<<endl;
        isRun_shakehand= false;
        return;
    }
    //2.机器人空闲中,执行去到抬手点位
    if(VCRRF->getStateMonitor().flag_rbCtlBusy){
        setTaskState("dealErr");
        cout<<"机器人不在空闲状态,进入处理故障状态"<<endl;
        isRun_shakehand= false;
        return;
    }
//    //3.机器人去到握手点位成功
    if(VCRRF->robGotoShakeHandPose()!=0){
        setTaskState("dealErr");
        cout<<"机器人去到握手点失败,进入处理故障状态"<<endl;
        isRun_shakehand= false;
        return;
    }
    //4.开启阻抗模式
    if(VCRRF->startImpedence()!=0){
        setTaskState("dealErr");
        cout<<"机器人开启阻抗失败,进入处理故障状态"<<endl;
        isRun_shakehand= false;
        return;
    }
    taskFlag_shakehand=true;
//    //5.等待握手结束,发信号控制机械臂回原点
//    while((!isStop)&&(!isErr))
//    {
//        if(VCRRF->getStateMonitor().isEnd_shakeHand)
//        {
////            //关闭阻抗
//            if(VCRRF->closeImpedence()!=0)
//            {
//                setTaskState("dealErr");
//                cout<<"机器人关闭阻抗失败,进入处理故障状态"<<endl;
//                isRun_shakehand= false;
//                return;
//            }
//
//            //回原点
//            if(VCRRF->RobGoHome()!=0)
//            {
//                setTaskState("dealErr");
//                cout<<"机器人回原点失败,进入处理故障状态"<<endl;
//                isRun_shakehand= false;
//                return;
//            }
//            break;
//        }
//        usleep(100);
//    }
    //正常完成握手流程,则继续切换回检测状态
    setTaskState("detection");
    isRun_shakehand= false;
}

void VoiceCtlRobTask::Shakehand_quiting(const std::vector<std::string> &args) {
    publishStateMsg(true,"quiting","-----shakehand_quiting------");
    while (isRun_shakehand){
        usleep(10);
    }
    publishStateMsg(true,"quiting","-----shakehand_quiting------");

    cout<<"Shakehand_initing结束"<<endl;
}

void VoiceCtlRobTask::transShakehand2exit(const std::vector<std::string> &args) {
    publishStateMsg(true,"toExit","-----shakehand_toExit------");
    isStop=true;
    setTaskState("exit");
}

void VoiceCtlRobTask::transShakehand2dealErr(const std::vector<std::string> &args) {
    publishStateMsg(true,"toDealErr","-----shakehand_toDealErr------");
    isErr=true;
    setTaskState("dealErr");
}

//挥手状态
void VoiceCtlRobTask::Wave_initing(const std::vector<std::string> &args) {
    publishStateMsg(true,"initing","-----wave_initing------");
    cout<<"挥手"<<endl;
    if(VCRRF->wave()!=0){
        cout<<"挥手失败"<<endl;
        setTaskState("dealErr");
    }
    setTaskState("detection");
}

void VoiceCtlRobTask::Wave_quiting(const std::vector<std::string> &args) {
    publishStateMsg(true,"quiting","-----wave_quiting------");
}

void VoiceCtlRobTask::transWave2exit(const std::vector<std::string> &args) {
    publishStateMsg(true,"toExit","-----wave_toExit------");
    isStop=true;
    setTaskState("exit");
}

void VoiceCtlRobTask::transWave2dealErr(const std::vector<std::string> &args) {
    publishStateMsg(true,"toDealErr","-----wave_toDealErr------");
    isErr=true;
    setTaskState("dealErr");
}

//检测娃娃状态
void VoiceCtlRobTask::DetectToy_initing(const std::vector<std::string> &args) {
    publishStateMsg(true,"initing","-----detectToy_initing------");
    //去到检测点
    if(VCRRF->detectPose()!=0){
        setTaskState("dealErr");
        return;
    }
    //进行yolo6d识别
    string name ="toy1";
    if(VCRRF->detect(name)!=0){
        cout<<"识别错误"<<endl;
        setTaskState("dealErr");
        return;
    }
    //等待识别到目标
    int wait_time=0;
    while (wait_time<5)
    {
        sleep(1);
        wait_time++;
        if(VCRRF->getStateMonitor().detectOk)
        {
            //识别成功则进行抓取切换
            setTaskState("grabToy");
            return;
        }
    }

}

void VoiceCtlRobTask::DetectToy_quiting(const std::vector<std::string> &args) {
    publishStateMsg(true,"quiting","-----detectToy_quiting------");
}

void VoiceCtlRobTask::transDetectToy2exit(const std::vector<std::string> &args) {
    publishStateMsg(true,"toExit","-----detectToy_toExit------");
    isStop=true;
    setTaskState("exit");
}

void VoiceCtlRobTask::transDetectToy2dealErr(const std::vector<std::string> &args) {
    publishStateMsg(true,"toDealErr","-----detectToy_toDealErr------");
    isErr=true;
    setTaskState("dealErr");
}

//抓娃娃状态
void VoiceCtlRobTask::grabToy_initing(const std::vector<std::string> &args) {
    publishStateMsg(true,"initing","-----grabToy_initing------");
    cout<<"抓娃娃"<<endl;
    geometry_msgs::PoseStamped a =VCRRF->getStateMonitor().object_pose;
    if(a.header.frame_id != "world"){
        cout<<"转换失败"<<endl;
        setTaskState("dealErr");
        return;
    }
    if(VCRRF->pick(a)!=0){
        cout<<"抓取失败"<<endl;
        setTaskState("dealErr");
        return;
    }
    geometry_msgs::PoseStamped b = VCRRF->placeObjPose[0];
    if(VCRRF->place(b)!=0){
        cout<<"放置失败"<<endl;
        setTaskState("dealErr");
        return;
    }
    VCRRF->RobGoHome();

    setTaskState("detection");
}

void VoiceCtlRobTask::grabToy_quiting(const std::vector<std::string> &args) {
    publishStateMsg(true,"quiting","-----grabToy_quiting------");
}

void VoiceCtlRobTask::transgrabToy2exit(const std::vector<std::string> &args) {
    publishStateMsg(true,"toExit","-----grabToy_toExit------");
    isStop=true;
    setTaskState("exit");
}

void VoiceCtlRobTask::transgrabToy2dealErr(const std::vector<std::string> &args) {
    publishStateMsg(true,"toDealErr","-----grabToy_toDealErr------");
    isErr=true;
    setTaskState("dealErr");
}

//故障处理状态
void VoiceCtlRobTask::dealErr_initing(const std::vector<std::string> &args) {
    cout<<"故障处理"<<endl;
    publishStateMsg(true,"initing","-----dealErr_initing------");


    setTaskState("exit");
}

void VoiceCtlRobTask::dealErr_quiting(const std::vector<std::string> &args) {
    publishStateMsg(true,"quiting","-----dealErr_quiting------");

}

//退出处理状态
void VoiceCtlRobTask::exit_initing(const std::vector<std::string> &args) {

    publishStateMsg(true,"initing","-----exit_initing------");
    //关闭声音
    VCRRF->VoiceDetect_Switch(false);
    //关闭行人检测
    VCRRF->PersonDetect_Switch(false);
    VCRRF->closeImpedence();
//    VCRRF->resetfunction();

    cout<<"退出完成"<<endl;
}

void VoiceCtlRobTask::exit_quiting(const std::vector<std::string> &args) {
    publishStateMsg(true,"quiting","-----exit_quiting------");
}

void VoiceCtlRobTask::transexit2init(const std::vector<std::string> &args) {
    publishStateMsg(true,"start","-----exit_start------");
    sleep(1);
    isErr= false;
    isStop=false;
    setTaskState("init");
}

void VoiceCtlRobTask::SetModeToSwithState(const char* state) {
    if(flag_AutoMode)
    {
        setTaskState(state);
    }

}

void VoiceCtlRobTask::publishStateMsg(bool status,std::string behevior,std::string meassage) {
    State sta;
    sta.status=status;
    sta.behevior=behevior;
    sta.meassage = {meassage};
    setRecallState(sta);
    notityRecall();
    usleep(1000);
}









