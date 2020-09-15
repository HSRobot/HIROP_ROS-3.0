#include <HsTaskFsmFactory.h>
#include <HPluginLoaderAggre.h>

#include <pickplacetask.h>

using namespace HSPlugin;

#define TASK_REGEX "(?<=lib)\\w+(?<=Task)"

#define LIB_SERACH_PATH "/home/fshs/catkin_ws/src/HIROP_ROS-3.0/build/devel/lib"

/**
 * @brief   LIB_SERACH_PATH_ENV
 *          获取插件搜索路径的环境变量名称
 */
#define LIB_SERACH_PATH_ENV "TASK_PLUGIN_PATH"

std::vector<std::string> taskList;


HPluginLoaderAggre* loadTool = HPluginLoaderAggre::getInstance();

HsTaskFsmFactory::HsTaskFsmFactory()
{
}

std::shared_ptr<FsmFramworkInterface> HsTaskFsmFactory::createByTest(ros::NodeHandle &nh, int type)
{
    std::shared_ptr<FsmFramworkInterface> framework = nullptr;
    std::shared_ptr<HsTaskFramework> ptr = nullptr;

    Task task = Task(type);
    switch (task) {
        case PickPlace:
        {
                ptr = std::make_shared<PickPlaceTask>();
        }
            break;
    case Unkown:
        assert(0 ==1 );
        break;
        default:
            break;
    }

    framework = std::make_shared<HsFsm::HsTaskFramework>(nh, ptr);
    std::cout <<framework->getTaskName()<<std::endl;
    return framework;

}

std::shared_ptr<FsmFramworkInterface> HsTaskFsmFactory::createByPlugin(ros::NodeHandle &nh, const string &taskName)
{
    auto it =  std::find(taskList.begin(), taskList.end(), taskName);
    if(it  == taskList.end())
    {
        std::cout<< " not find "<<taskName<<" plugin "<<std::endl;
        return nullptr;
    }

    FsmFramworkInterface * temp = (FsmFramworkInterface *)loadTool->loadClassPlugin<FsmFramworkInterface>(taskName);
    if(temp == nullptr)
        return nullptr;
    HsTaskFramework *ptr = dynamic_cast<HsTaskFramework*>(temp);
    std::shared_ptr<HsTaskFramework> sPtr(ptr);
    return std::make_shared<HsTaskFramework>(nh, sPtr);
}


std::vector<std::string> HsTaskFsmFactory::getClassPluginList()
{
    int ret = setenv(LIB_SERACH_PATH_ENV, LIB_SERACH_PATH,0);
    assert(ret == 0);

    loadTool->setSearchPath(LIB_SERACH_PATH_ENV);
//    std::cout << "LIB_SERACH_PATH : "<<LIB_SERACH_PATH<<std::endl;

    taskList.clear();
    loadTool->getClassPluginList(taskList, TASK_REGEX);

    return taskList;
}
