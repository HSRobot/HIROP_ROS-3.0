#include "VisualCaptureRosTask.h"

using namespace HsFsm;
using namespace std;

typedef std::pair<string, int> elment;

VisualCaptureRosTask::VisualCaptureRosTask(const string &taskName)
{
    this->taskName = taskName;
    VCRF = new VisualCaptureRosFunc(getRosHandler());
    ErrInfoList={"重新初始化后机器人上使能失败","回原点失败","点云生成失败","抓取失败","机器人规划运动到识别点失败",
    "坐标转换失败","放置失败"};

}

VisualCaptureRosTask::~VisualCaptureRosTask()
{
    delete VCRF;
}

void VisualCaptureRosTask::init()
{
    // 自动转为 init 状态
    // 用户自定义添加内容
    cout << "已自动转为init状态" << endl;
}

void VisualCaptureRosTask::quit()
{
    //会自动转状态为 quit
    // 用户自定义添加内容
    cout << "已自动转为quit状态" << endl;
}

bool VisualCaptureRosTask::registerTaskList()
{
    //基本状态行为函数
    auto func_init_initing = bind(&VisualCaptureRosTask::init_initing, this, placeholders::_1);
    auto func_init_quiting = bind(&VisualCaptureRosTask::init_quiting, this, placeholders::_1);
    auto func_init_starting = bind(&VisualCaptureRosTask::init_starting, this, placeholders::_1);

    auto func_prepare_initing = bind(&VisualCaptureRosTask::prepare_initing, this, placeholders::_1);
    auto func_prepare_quiting = bind(&VisualCaptureRosTask::prepare_quiting, this, placeholders::_1);

    auto func_look_initing = bind(&VisualCaptureRosTask::look_initing, this, placeholders::_1);
    auto func_test_initing = bind(&VisualCaptureRosTask::test_initing, this, placeholders::_1);
    auto func_look_quiting = bind(&VisualCaptureRosTask::look_quiting, this, placeholders::_1);

    auto func_detection_initing = bind(&VisualCaptureRosTask::detection_initing, this, placeholders::_1);
    auto func_detection_quiting = bind(&VisualCaptureRosTask::detection_quiting, this, placeholders::_1);
    auto func_pick_initing = bind(&VisualCaptureRosTask::pick_initing, this, placeholders::_1);
    auto func_pick_quiting = bind(&VisualCaptureRosTask::pick_quiting, this, placeholders::_1);
    auto func_place_initing = bind(&VisualCaptureRosTask::place_initing, this, placeholders::_1);
    auto func_place_quiting = bind(&VisualCaptureRosTask::place_quiting, this, placeholders::_1);
    auto func_error_initing = bind(&VisualCaptureRosTask::error_initing, this, placeholders::_1);
    auto func_error_quiting = bind(&VisualCaptureRosTask::error_quiting, this, placeholders::_1);
    auto func_exit_initing = bind(&VisualCaptureRosTask::exit_initing, this, placeholders::_1);
    auto func_exit_quiting = bind(&VisualCaptureRosTask::exit_quiting, this, placeholders::_1);
setTaskState("error");
    //手动状态切换行为函数
    auto func_transPrepare2Detection = bind(&VisualCaptureRosTask::transPrepare2Detection, this, placeholders::_1);
    auto func_transPrepare2Exit = bind(&VisualCaptureRosTask::transPrepare2Exit, this, placeholders::_1);
    auto func_transDetection2Exit = bind(&VisualCaptureRosTask::transDetection2Exit, this, placeholders::_1);
    auto func_transPick2Exit = bind(&VisualCaptureRosTask::transPick2Exit, this, placeholders::_1);
    auto func_transPlace2Exit = bind(&VisualCaptureRosTask::transPlace2Exit, this, placeholders::_1);
    auto func_transExit2Init = bind(&VisualCaptureRosTask::transExit2Init, this, placeholders::_1);

    try{
        registerTask("init", "initing", func_init_initing);
        registerTask("init", "quiting", func_init_quiting);
        registerTask("init", "starting", func_init_starting);

        registerTask("prepare", "initing", func_prepare_initing);
        registerTask("prepare", "quiting", func_prepare_quiting);
        registerTask("prepare", "starting", func_transPrepare2Detection);
        registerTask("prepare", "exit", func_transPrepare2Exit);

        registerTask("look", "initing", func_look_initing);
        registerTask("test", "initing", func_test_initing);
        registerTask("look", "quiting", func_look_quiting);

        registerTask("detection", "initing", func_detection_initing);
        registerTask("detection", "quiting", func_detection_quiting);
        registerTask("detection", "exit", func_transDetection2Exit);

        registerTask("pick", "initing", func_pick_initing);
        registerTask("pick", "quiting", func_pick_quiting);
        registerTask("pick", "exit", func_transPick2Exit);

        registerTask("place", "initing", func_place_initing);
        registerTask("place", "quiting", func_place_quiting);
        registerTask("place", "exit", func_transPlace2Exit);

        registerTask("error", "initing", func_error_initing);
        registerTask("error", "quiting", func_error_quiting);

        registerTask("exit", "initing", func_exit_initing);
        registerTask("exit", "quiting", func_exit_quiting);
        registerTask("exit", "restart", func_transExit2Init);

    }catch (exception &e){
        cout << e.what() << endl;
        assert(-1);
        return false;
    }

    return true;
}


/***************状态行为函数*****************/

//init状态
void VisualCaptureRosTask::init_initing(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "initing", "------init_initing------");
    sleep(1);
    typeCode = 0;
    taskRunStatus = true;
    isForceQuit = false;
    cout << "VisionCapture状态机已进入init状态" << endl;

    //VCRF->StateMonitorInit();

    setTaskState("prepare");
}

void VisualCaptureRosTask::init_starting(const std::vector<std::string> &args){
    setTaskState("prepare");
}


void VisualCaptureRosTask::init_quiting(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "quiting", "------init_quiting------");
    cout << "VisionCapture状态机已退出init状态" << endl;
}

//prepare状态
void VisualCaptureRosTask::prepare_initing(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "initing", "------prepare_initing------");
    cout << "VisionCapture状态机已进入prepare状态" << endl;
    if(!VCRF->getPedsDetectionState())
    {
        // cout << VCRF->creatPedsDetection() << endl;
        ros::Duration(2).sleep();
    }
    //启动prepare状态运行标志位
    isPreparing = true;

    //bool robot_enable = true;
    VCRF->RobotEnable(true);

    if (!VCRF->getStateMonitor().RobotStatus && !VCRF->getStateMonitor().RobotEnableStatus){
        cout << "重新初始化后机器人上使能失败！" << endl;
        index_errinfo=0;
        setTaskState("error");
        isPreparing = false;
        return;
    }else{
        cout << "重新初始化后机器人上使能成功！" << endl;
    }

    if(VCRF->RobotGoHome() != 0){
        index_errinfo=1;
        setTaskState("error");
    }



    isPreparing = false;

    setTaskState("look");
}

void VisualCaptureRosTask::prepare_quiting(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "quiting", "------prepare_quiting------");
    while(isPreparing){
        usleep(10);
    }
    cout << "VisionCapture状态机已退出prepare状态" << endl;
}

void VisualCaptureRosTask::transPrepare2Detection(const std::vector<std::string> &args)
{
    // VCRF->shutdownPedsDetection();
    VisualCaptureRosTask::publishStateMsg(true, "starting", "------prepare_starting------");
    detect_object = args[0];
    setTaskState("look");
}

void VisualCaptureRosTask::transPrepare2Exit(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "exit", "------prepare_exit------");
    setTaskState("exit");
    isForceQuit = true;
}


//look 状态
void VisualCaptureRosTask::look_initing(const std::vector<std::string> &args)
{
    cout << "VisionCapture状态机已进入look状态" << endl;
    publishStateMsg(true, "initing", "------look_initing------");

    isLooking = true;
    int result;

    //关闭行人检测
    while (VCRF->getPedsDetectionState())
    {
        std::cout << "等待行人检测关闭" << std::endl;
    }

    //场景构建
    for (int i = 0; i < 2; i++){
        // ros::Duration(2).sleep();
        //if(result = VCRF->lookPointCloud())
        //{
        //    index_errinfo=2;
            //setTaskState("error");
        //    isLooking = false;
            //return;
        //}
    }
    detect_object = "wangzai1";
    if(detect_object == "测试")
        setTaskState("test");
    else
    {
        result = 0;
        if(result == 0)
            setTaskState("detection");
        else
            setTaskState("error");
    }

    // VCRF->creatPedsDetection();

    isLooking = false;
}

void VisualCaptureRosTask::test_initing(const std::vector<std::string> &args)
{
    vector<geometry_msgs::PoseStamped> pose;
    if(VCRF->loadRobotPose(pose, "pick") == 0)
    {
        if (VCRF->RobotPick(pose[0]) != 0)
        {
            index_errinfo=3;
            cout << "抓取失败" << endl;
            setTaskState("error");
            return;
        }
        else
        {
            cout << "抓取成功" << endl;
            setTaskState("place");
        }
    }
    else
    {
        cout << "加载点位失败" << endl;
    }
}

void VisualCaptureRosTask::look_quiting(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "quiting", "------look_quiting------");
    while(isLooking){
        usleep(10);
    }
    cout << "VisionCapture状态机已退出look状态" << endl;
}







//detection状态
void VisualCaptureRosTask::detection_initing(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "initing", "------detection_initing------");
    cout << "VisionCapture状态机已进入detection状态" << endl;

    //启动detection运行标志位
    isDetecting = true;

    //机器人去到识别点
    if (VCRF->RobotGoDetection() != 0){
        index_errinfo=4;
        setTaskState("error");
        cout << "机器人规划运动到识别点失败" << endl;
        isDetecting = false;
        return;
    }

//    setTaskState("init");
//    isDetecting = false;
//    return;
    
    //开始进行Yolo6d物体识别
    
    if (VCRF->detect(detect_object) != 0){
        VCRF->RobotGoHome();
        index_errinfo=1;
        setTaskState("error");
        isDetecting = false;
        return;
    }

    //等待抓取目标pose
    int wait_time = 0;
    while (wait_time < 5){
        sleep(1);
        wait_time++;
        if (VCRF->getStateMonitor().havePickPose){
            //已接收到抓取目标pose,进行抓取
            setTaskState("pick");
            isDetecting = false;
            return;
        }
    }
}

void VisualCaptureRosTask::detection_quiting(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "quiting", "------detection_quiting------");
    while(isDetecting){
        usleep(10);
    }
    cout << "VisionCapture状态机已退出detection状态" << endl;
}

void VisualCaptureRosTask::transDetection2Exit(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "exit", "------detection_exit------");
    setTaskState("exit");
    isForceQuit = true;
}

//pick状态
void VisualCaptureRosTask::pick_initing(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "initing", "------pick_initing------");
    cout << "VisionCapture状态机已进入pick状态" << endl;
    
    //启动pick状态运行标志位
    isPicking = true;

    cout << "机器人准备开始抓娃娃" << endl;
    geometry_msgs::PoseStamped a = VCRF->PickObjPoses[0];

    //先判断坐标是否转换成功
    if (a.header.frame_id != "world"){
        cout << "坐标转换失败" << endl;
        index_errinfo=5;
        setTaskState("error");
        isPicking = false;
        return;
    }

    //进行抓取
    if (VCRF->RobotPick(a) != 0){
        cout << "抓取失败" << endl;
        index_errinfo=3;
        setTaskState("error");
        isPicking = false;
        return;
    }else{
        cout << "抓取成功" << endl;
    }

    setTaskState("place");
    isPicking = false;
}

void VisualCaptureRosTask::pick_quiting(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "quiting", "------pick_quiting------");
    while(isPicking){
        usleep(10);
    }
    cout << "VisionCapture状态机已退出pick状态" << endl;
}

void VisualCaptureRosTask::transPick2Exit(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "exit", "------pick_exit------");
    setTaskState("exit");
    isForceQuit = true;
}

//place状态
void VisualCaptureRosTask::place_initing(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "initing", "------place_initing------");
    cout << "VisionCapture状态机已进入places状态" << endl;

    //启动place状态运行标志位
    isPlacing = true;

    geometry_msgs::PoseStamped b = VCRF->PlaceObjPoses[0];

    if (VCRF->RobotPlace(b) != 0){
        cout << "放置失败" << endl;
        index_errinfo=6;
        setTaskState("error");
        isPlacing = false;
        return;
    }else{
        cout << "放置成功" << endl;
    }

    //放置成功后返回原点
    VCRF->RobotGoHome();
    setTaskState("prepare");
    isPlacing = false;
    
}

void VisualCaptureRosTask::place_quiting(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "quiting", "------place_quiting------");
    while(isPlacing){
        usleep(10);
    }
    cout << "VisionCapture状态机已退出place状态" << endl;

}

void VisualCaptureRosTask::transPlace2Exit(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "exit", "------place_exit------");
    setTaskState("exit");
    isForceQuit = true;
}

//error状态
void VisualCaptureRosTask::error_initing(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "initing",ErrInfoList[index_errinfo]);
    cout << "VisionCapture状态机已进入error状态" << endl;

    //启动error状态运行标志位
    isError = true;
    //机器人下使能并停止下发点位
    // VCRF->RobotEnable(false);
    VCRF->RobotStopMotion();

    setTaskState("exit");
    isError = false;
}

void VisualCaptureRosTask::error_quiting(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "quiting", "------error_quiting------");
    while(isError){
        usleep(10);
    }
    cout << "VisionCapture状态机已退出error状态" << endl;
}

//exit状态
void VisualCaptureRosTask::exit_initing(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "initing", "------exit_initing------");
    cout << "VisionCapture状态机已进入exit状态" << endl;

    //如果是强制退出，则需自行下使能
    if (isForceQuit){
        // VCRF->RobotEnable(false);
        isForceQuit = false;
    }

    setTaskState("init");

}

void VisualCaptureRosTask::exit_quiting(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "quiting", "------exit_quiting------");
    cout << "VisionCapture状态机已退出exit状态" << endl;

}

void VisualCaptureRosTask::transExit2Init(const std::vector<std::string> &args)
{
    VisualCaptureRosTask::publishStateMsg(true, "restart", "------exit_restart------");
    VCRF->RobotReset();
    setTaskState("init");
}

/******************************************/


void VisualCaptureRosTask::publishStateMsg(bool status, string behaviour, string message)
{
    State sta;
    sta.status = status;
    sta.behevior = behaviour;
    sta.meassage = {message};
    setRecallState(sta);
    notityRecall();
    usleep(1000);
}
