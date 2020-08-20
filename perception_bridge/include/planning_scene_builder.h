#pragma once
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include "ros/package.h"

#include "moveit/move_group_interface/move_group_interface.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/PlanningScene.h"
#include "moveit_msgs/ObjectColor.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include <std_msgs/ColorRGBA.h>

#include <hirop_msgs/Pose.h>

#include <vector>
#include <string.h>
#include <map>
#include <stdlib.h>
#include <stdexcept>
#include <geometry_msgs/Quaternion.h>

using namespace std;


class PlanningSceneBuilder
{
private:
    ros::Publisher pub_scene_diff;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::CollisionObject getMesh(string sys, string name, string frame_id, geometry_msgs::Pose pose);

    geometry_msgs::Quaternion rpy2Quaternion(double r, double p, double y);

    moveit_msgs::ObjectColor setColor(string id, double r, double g, double b, double a);

    geometry_msgs::Pose setPose(double px, double py, double pz, double r, double p, double y);

    /**
     * @brief 检查是否发布物体是否发布成功
     * @param id 发布物体的id的list
     * @return 全部发布成功,返回true
    */
    bool checkPubStatus(vector<string> id);

public:
    /**
     * @brief 发布mesh模型
     * @param file 发布模型的文件名, 放在功能包的mesh下
     * @param id 发布到规划场景中使用的id
     * @param frame_id 参考系
     * @param color 物体的颜色
     * @param pose 模型的姿态
     * @return 是否发布成功
    */
    bool pubMesh(std::string& file, std::string& id, std::string frame_id, \
                std_msgs::ColorRGBA& color, hirop_msgs::Pose& pose);

    /**
     * @brief 发布基础的几何物体
     * @param frame_id 参考系
     * @param shape 物体的形状和大小
     * @param pose 发布到
    */
    bool pubCollisionObject(std::string& frame_id, vector<string> id, vector<shape_msgs::SolidPrimitive>& shape, \
                            vector<hirop_msgs::Pose>& pose, vector<std_msgs::ColorRGBA>& color);

    bool removeObject(std::string& id);
    PlanningSceneBuilder(ros::Publisher& );
    ~PlanningSceneBuilder();
};


