#include <HPluginLoaderAggre.h>
#include <gtest/gtest.h>


#define TASK_REGEX "(?<=lib)\\w+(?<=Task)"

#define LIB_SERACH_PATH "/home/fshs/catkin_ws/src/HIROP_ROS-3.0/build/devel/lib"

/**
 * @brief   LIB_SERACH_PATH_ENV
 *          获取插件搜索路径的环境变量名称
 */
#define LIB_SERACH_PATH_ENV "TASK_PLUGIN_PATH"

 using namespace HSPlugin;
class TestMap : public testing::Test
{
public:
    TestMap(){}

    void SetUp()
    {
//        plugin = HPluginLoaderAggre::getInstance();
    }

    void  TearDown()
    {

    }
private:
    std::vector<std::string> taskList ;

};


TEST_F(TestMap, FindPlugin)
{
    HPluginLoaderAggre* plugin =  HPluginLoaderAggre::getInstance();

    plugin->setSearchPath(LIB_SERACH_PATH_ENV);

    plugin->getClassPluginList(taskList, TASK_REGEX);
}

TEST_F(TestMap, SetTask)
{
    HPluginLoaderAggre* plugin =  HPluginLoaderAggre::getInstance();
}




int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
