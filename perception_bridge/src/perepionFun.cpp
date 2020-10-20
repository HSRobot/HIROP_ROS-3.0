#include "perepionFun.h"

#include "hirop/perception/pclFilterFactory.h"
#include "hirop/perception/PclFilterManage.h"
#include "hirop/perception/PclFilterManageSingle.h"
#include "hirop/perception/transformUltity.h"
#include "pcl/common/io.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/conversions.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>


#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <thread>

using namespace std;

typedef pcl::PointXYZRGB TEMPPOINT;

PclFilterManager<TEMPPOINT>* manager;

moveit_msgs::PlanningScene planning_scene;
moveit_msgs::PlanningSceneWorld planning_scene_world;


pereptionFun::pereptionFun(ros::NodeHandle &n)
{
    node = n;
    havePointCloud = false;

    node.param("pointCloudTopic", pclReceviTopicName,string("/kinect2/hd/points"));
    node.param("referFrame", referFrame, string("/base_link"));
    node.param("baseFrame", baseFrame, string("/camera"));
    node.param("publishPclName", publishPclName, string("/filter_points"));

    manager = PclFilterManagerSingle<TEMPPOINT>::getInstance()->getManager();
//    auto it1 = pclFilterFactory<TEMPPOINT >::create(PCLFILTER::object);
    auto it2 = pclFilterFactory<TEMPPOINT >::create(PCLFILTER::voxel);
    auto it3 = pclFilterFactory<TEMPPOINT >::create(PCLFILTER::region);

    // list connect
//    manager->addFilterEntry(it1);
    manager->addFilterEntry(it2);
    manager->addFilterEntry(it3);

    initRosTopic();
    planningScenePtr = new PlanningSceneBuilder(publishOctomapPub);
}

void pereptionFun::start()
{
    ros::MultiThreadedSpinner p;
    p.spin();
}


void pereptionFun::initRosTopic()
{
    publishPclCloudPub = node.advertise<sensor_msgs::PointCloud2>("/filter_points", 1);
    //publishOctomapPub = node.advertise<moveit_msgs::PlanningSceneWorld>("/planning_scene_world", 1);
	publishOctomapPub = node.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    lookServer = node.advertiseService("/lookPointCloud",&pereptionFun::lookPointCloudCB, this);
    clearServer = node.advertiseService("/clearScene", &pereptionFun::clearSceneCB, this);
    loadPCServer = node.advertiseService("/loadPointCloud",&pereptionFun::loadPointCloudCB, this);
    savePCServer = node.advertiseService("/savePointCloud",&pereptionFun::savePointCloudCB, this);
	updateServer = node.advertiseService("/updatePointCloud",&pereptionFun::updatePointCloudCB, this);
    transformPoseServer = node.advertiseService("/transformPose", &pereptionFun::transformPoseCB, this);
    moveit_clear_octomap_client = node.serviceClient<std_srvs::Empty>("clear_octomap");
	lookClient = node.serviceClient<hirop_msgs::LookPointCloud>("/lookPointCloud");
	clearClient = node.serviceClient<hirop_msgs::ClearScene>("/clearScene");

    /****/
    pubMeshServer = node.advertiseService("/loadMesh", &pereptionFun::pubMeshServerCB, this);
    pubObjectServer = node.advertiseService("/loadObject", &pereptionFun::pubObjectServerCB, this);
    rmObject = node.advertiseService("rmObject", &pereptionFun::rmObjectCB, this);
    /****/
}

bool pereptionFun::lookPointCloudCB(hirop_msgs::LookPointCloud::Request &req,
                                     hirop_msgs::LookPointCloud::Response &res)
{
    pointCloudSub = node.subscribe<sensor_msgs::PointCloud2>(pclReceviTopicName, 1, \
                                                                 &pereptionFun::pointCloudCallBack, this);
    for(int i = 0; i < 5; i++){
        if(!havePointCloud){
            ros::Duration(1.0).sleep();
//            IDebug("Waiting for point cloud");
        }
    }

    /**
     *  还未接收到点云，重复执行当前过滤器
     */
    if(!havePointCloud){
        res.reuslt = -1;
        return false;
    }

    //** process
    process();

//    transformUltity::transformFrame(baseFrame ,referFrame, T,R);

    sensor_msgs::PointCloud2 msg;

#if PCL_VERSION_COMPARE(<,1,7,0)
    ::pcl::toROSMsg(*input, *msg);
#else
    pcl::PCLPointCloud2 pcd_tmp;
    boost::shared_ptr<pcl::PointCloud<TEMPPOINT> > point;
    manager->getOutCloud(point);
    pcl::toPCLPointCloud2(*point.get(), pcd_tmp);
    pcl_conversions::fromPCL(pcd_tmp, msg);
#endif

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = cameraFrame;
    publishPclCloudPub.publish(msg);

    /******离群点处理******/
    pcl::PointCloud<TEMPPOINT>::Ptr cloud_filter_or(new pcl::PointCloud<TEMPPOINT>);
    pcl::StatisticalOutlierRemoval<TEMPPOINT> outlier_removal_sor;
    outlier_removal_sor.setInputCloud(point);
	outlier_removal_sor.setMeanK(200); //K近邻搜索点个数
	outlier_removal_sor.setStddevMulThresh(1.0); //标准差倍数
	//outlier_removal_sor.setNegative(false); //保留未滤波点（内点）
    outlier_removal_sor.setKeepOrganized(true);
	outlier_removal_sor.filter(*cloud_filter_or);  //保存滤波结果到cloud_filter
    /************************/

    cloud_filter_or->width = 1;
    cloud_filter_or->height = cloud_filter_or->points.size();

    /**转换矩阵**/
     pcl::PointCloud<TEMPPOINT>::Ptr cloud_trans(new pcl::PointCloud<TEMPPOINT>);
    transformUltity::transformFrame("world", cameraFrame, T, R);
    Eigen::Isometry3d trans_matrix = transformUltity::makeMatrix(T, R);
    pcl::transformPointCloud(*cloud_filter_or, *cloud_trans, trans_matrix.matrix());
    /*******/

    /**
      pcl转octomap
    */
    std::cout << "copy data into octomap..." << std::endl;
    octomap::ColorOcTree tree( 0.05 );
    tree.clear();

    if (cloud_trans->size() == 0){
        cout << "点云过滤后数量为0,无法八叉树化,请重新设置点云过滤配置参数" << endl;
        return false;
    }

    //将点云的点插入到八叉树种
    // for (auto p:*point.get()){
    for (auto p:cloud_trans->points){
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }

    //设置颜色信息
    // for (auto p:*point.get()){
    for (auto p:cloud_trans->points){
        tree.integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }

    //更新octomap
    tree.updateInnerOccupancy();
    tree.write("/home/de/catkin_ws_v3/src/HIROP_ROS-3.0/perception_bridge/data/live_octomap.ot");

    //转换成octomap_msgs消息格式
    static octomap_msgs::Octomap octomap;
    octomap_msgs::binaryMapToMsg(tree, octomap);

    //planning_scene_world.octomap.header.frame_id = "world";
    //planning_scene_world.octomap.header.stamp = ros::Time::now();
    //planning_scene_world.octomap.octomap.header.frame_id = "world";
    //planning_scene_world.octomap.octomap.header.stamp = ros::Time::now();
    //planning_scene_world.octomap.octomap.binary = true;
    //planning_scene_world.octomap.octomap.id = "OcTree";
    //planning_scene_world.octomap.octomap.data = octomap.data;
    //planning_scene_world.octomap.octomap.resolution = 0.05;


    planning_scene.world.octomap.header.stamp = ros::Time::now();
	planning_scene.world.octomap.header.frame_id = "world";
    planning_scene.world.octomap.octomap.header.stamp = ros::Time::now();
    planning_scene.world.octomap.octomap.header.frame_id = "world";
    planning_scene.world.octomap.octomap.binary = true;
    planning_scene.world.octomap.octomap.id = "OcTree";
    planning_scene.world.octomap.octomap.data = octomap.data;
    planning_scene.world.octomap.octomap.resolution = 0.05;
	planning_scene.is_diff = true;

    //发布octomap到planning_scene_world
    //publishOctomapPub.publish(planning_scene_world);
	publishOctomapPub.publish(planning_scene);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ROS_INFO("Requset: build the Octomap");
    if (publishOctomapPub.getNumSubscribers() < 1){
        ROS_INFO("No subscriber, waitting...");
    }else{
        ROS_INFO("Find the subscriber,strat publishing msgs and on...");
    }

    res.reuslt = 0;
    return true;
}

bool pereptionFun::clearSceneCB(hirop_msgs::ClearScene::Request &req,
                      hirop_msgs::ClearScene::Response &res)
{
    pcl::PointCloud<pcl::PointXYZ> clean_cloud;
    pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/de/catkin_ws_v3/src/HIROP_ROS-3.0/perception_bridge/data/clean.pcd", clean_cloud);

    octomap::ColorOcTree tree(0.01);
    tree.clear();

    //将点云的点插入到八叉树种
    for (auto p:clean_cloud.points){
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }

    tree.updateInnerOccupancy();
    static octomap_msgs::Octomap octomap_clean_msg;
    octomap_msgs::binaryMapToMsg(tree, octomap_clean_msg);

    planning_scene.world.octomap.header.frame_id = cameraFrame;
    planning_scene.world.octomap.header.stamp = ros::Time::now();
    planning_scene.world.octomap.octomap.header.frame_id = cameraFrame;
    planning_scene.world.octomap.octomap.header.stamp = ros::Time::now();
    planning_scene.world.octomap.octomap.binary = true;
    planning_scene.world.octomap.octomap.id = "OcTree";
    planning_scene.world.octomap.octomap.data = octomap_clean_msg.data;
    planning_scene.world.octomap.octomap.resolution = 0.05;
	planning_scene.is_diff = true;

    publishOctomapPub.publish(planning_scene);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std_srvs::Empty moveit_clear_octomap_srv;
    
    if (moveit_clear_octomap_client.call(moveit_clear_octomap_srv)){
        ROS_INFO("The scene has been clean up!");
        res.result = 0;
    }else{
        ROS_ERROR("Please set up moveit!!!");
        res.result = -1;
    }
  
    return true;
}

bool pereptionFun::updatePointCloudCB(hirop_msgs::UpdatePCL::Request &req,
						hirop_msgs::UpdatePCL::Response &res)
{
	hirop_msgs::LookPointCloud look_srv;
	hirop_msgs::ClearScene clear_srv;
	if (clearClient.call(clear_srv)){
		if (lookClient.call(look_srv)){
			res.result = 0;}
	}else{
		res.result = -1;
	}

	return true;
}

bool pereptionFun::loadPointCloudCB(hirop_msgs::LoadPCL::Request &req,
                      hirop_msgs::LoadPCL::Response &res)
{
    loadPCD(string(req.fileName));
    return true;
}


bool pereptionFun::savePointCloudCB(hirop_msgs::SavePCL::Request &req,
                      hirop_msgs::SavePCL::Response &res)
{
    savePCD(string(req.fileName));
    return true;
}

void pereptionFun::loadLocalParam(const string &path)
{
    yamlNode = YAML::LoadFile(path);
    manager->updatePxaram(yamlNode);

}


void pereptionFun::loadPCD(const string &path)
{
    assert(manager->loadPointCloud(path));
}

void pereptionFun::savePCD(const string &path)
{
    manager->savePointCloud("save",path);
}

int pereptionFun::process()
{
    int ret = manager->filterProcess();

    if(ret != 0)
        return false;
    boost::shared_ptr<pcl::PointCloud<TEMPPOINT> > outPoint;
    manager->getOutCloud(outPoint);
    std::cout<<"outPoint: " << outPoint->size() << std::endl;
    return ret;
}

void pereptionFun::printInfo()
{
   manager->printInfo();
}

void pereptionFun::clearPoint()
{
    sensor_msgs::PointCloud2 output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = baseFrame;
    publishPclCloudPub.publish(output);
}

void pereptionFun::pointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
//    IDebug("Got point cloud");
    pointCloudSub.shutdown();

    havePointCloud = false;
    cameraFrame = msg->header.frame_id;
    std::cout << "cameraFrame: "<< cameraFrame <<std::endl;

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    // msg to  PCL2
    pcl_conversions::toPCL(*msg, *cloud);


    /**
     * @brief pcl::fromPCLPointCloud2   将PointCloud2 转换为 PointCloud
     */
    pcl::PointCloud<TEMPPOINT>::Ptr pcl_cloud (new pcl::PointCloud<TEMPPOINT>);
    pcl::fromPCLPointCloud2(*cloud, *pcl_cloud);


    manager->setInputPCLPoint(*pcl_cloud);


    delete cloud;

    havePointCloud = true;
}

/****/
bool pereptionFun::pubMeshServerCB(hirop_msgs::PubMesh::Request& req, hirop_msgs::PubMesh::Response& rep)
{
    bool flag;
    ROS_INFO_STREAM("pub mesh collision_objects: " << req.object_id);
    flag = planningScenePtr->pubMesh(req.file_name, req.object_id, req.header.frame_id, \
                                req.rgba, req.pose);
    rep.result = flag;
    return flag;
}

bool pereptionFun::pubObjectServerCB(hirop_msgs::PubObject::Request& req, hirop_msgs::PubObject::Response& rep)
{
    bool flag;
    flag = planningScenePtr->pubCollisionObject(req.header.frame_id, req.object_id, \
                                                req.primitives, req.pose, req.rgba);
    rep.result = flag;
    return flag;
}

bool pereptionFun::rmObjectCB(hirop_msgs::removeObject::Request& req, hirop_msgs::removeObject::Response& rep)
{
    bool flag;
    ROS_INFO_STREAM("remove collision_objects: " << req.id);
    flag = planningScenePtr->removeObject(req.id);
    rep.result = flag;
    return flag;
}

bool pereptionFun::transformPoseCB(hirop_msgs::transformFramePose::Request& req, hirop_msgs::transformFramePose::Response& rep)
{
    geometry_msgs::PoseStamped target;
    if(transformFrame(req.sourcePose,target, req.targetFrame))
    {
        rep.targetPose = target;
        rep.isSucceed = true;
    }
    else
        rep.isSucceed = false;
    return true;
}

bool pereptionFun::transformFrame(const geometry_msgs::PoseStamped &p, geometry_msgs::PoseStamped &target,const string &frame_id)
{
//    geometry_msgs::PoseStamped *target_pose = new geometry_msgs::PoseStamped[1];
//    geometry_msgs::PoseStamped *source_pose = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped source_pose;

    tf::TransformListener tf_listener;

    source_pose = p;
    std::cout << " camera_color_optical_frame: "<< frame_id<<std::endl;
    for (int i = 0; i < 5; ++i)
    {
        try
        {
            tf_listener.transformPose(frame_id, source_pose, target_pose);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("transfrom exception : %s", ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
    }
    target = target_pose;

//    delete[] target_pose;
//    delete[] source_pose;

    if (target.header.frame_id == frame_id)
    {
        return true;
    }
    else
    {
        return false;
    }
}
/****/
