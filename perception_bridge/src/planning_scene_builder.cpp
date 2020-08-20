#include "planning_scene_builder.h"

PlanningSceneBuilder::PlanningSceneBuilder(ros::Publisher& pub)
:pub_scene_diff{pub}
{
}

PlanningSceneBuilder::~PlanningSceneBuilder()
{
}

moveit_msgs::CollisionObject PlanningSceneBuilder::getMesh(string sys, string name, string frame_id, geometry_msgs::Pose pose)
{
    // string path = "package://" + string("perception_bridge/meshes/") + sys + ".*";
    string path;
    path = "package://" + string("perception_bridge/mesh/") + sys + string(".stl");
    ROS_INFO_STREAM(path);
    shapes::Mesh* m = shapes::createMeshFromResource(path);
    moveit_msgs::CollisionObject object;
    shapes::ShapeMsg objectMeshMsg;
    shapes::constructMsgFromShape(m, objectMeshMsg);
    object.meshes.resize(1);
    object.meshes[0] = boost::get<shape_msgs::Mesh>(objectMeshMsg);
    object.header.frame_id = frame_id;
    object.id = name;
    object.mesh_poses.resize(1);
    object.mesh_poses[0] = pose;
    return object;
}

geometry_msgs::Quaternion PlanningSceneBuilder::rpy2Quaternion(double r, double p, double y)
{
    tf2::Quaternion quat;
    quat.setRPY(r, p, y);
    return tf2::toMsg(quat);
}

moveit_msgs::ObjectColor PlanningSceneBuilder::setColor(string id, double r, double g, double b, double a)
{
    moveit_msgs::ObjectColor color;
    color.id = id;
    color.color.r = r; 
    color.color.g = g;
    color.color.b = b; 
    color.color.a = a;
    return color;
}

bool PlanningSceneBuilder::pubMesh(string& file, string& id, string frame_id, \
                                    std_msgs::ColorRGBA& color, hirop_msgs::Pose& pose)
{
    moveit_msgs::PlanningScene p;
    geometry_msgs::Pose meshPose;
    meshPose.position = pose.position;
    meshPose.orientation = rpy2Quaternion(pose.rpy.R, pose.rpy.P, pose.rpy.Y);
    p.world.collision_objects.resize(1);
    p.world.collision_objects[0] = getMesh(file, id, frame_id, meshPose);
    p.object_colors.resize(1);
    p.object_colors[0] = setColor(id, color.r, color.g, color.b, color.a);
    p.world.collision_objects[0].operation = p.world.collision_objects[0].ADD;
    p.is_diff = true;
    pub_scene_diff.publish(p);
    ros::Duration(0.5).sleep();
    vector<string> idList;
    idList.push_back(id);
    return checkPubStatus(idList);
}

geometry_msgs::Pose PlanningSceneBuilder::setPose(double px, double py, double pz, double r, double p, double y)
{
    geometry_msgs::Pose pose;
    pose.position.x = px;
    pose.position.y = py;
    pose.position.z = pz;
    pose.orientation = rpy2Quaternion(r, p, y);
    return pose;
}

bool PlanningSceneBuilder::pubCollisionObject(std::string& frame_id, vector<string> id, vector<shape_msgs::SolidPrimitive>& shape, \
                                                vector<hirop_msgs::Pose>& pose, vector<std_msgs::ColorRGBA>& color)
{
    moveit_msgs::PlanningScene p;
    p.world.collision_objects.resize(id.size());
    p.object_colors.resize(id.size());
    for(int i=0; i<id.size(); i++)
    {
        ROS_INFO_STREAM("pub collision_objects: " << id[i]);
        p.world.collision_objects[i].header.frame_id = frame_id;
        p.world.collision_objects[i].id = id[i];
        p.world.collision_objects[i].primitives.push_back(shape[i]);
        geometry_msgs::Pose objPose = setPose(pose[i].position.x, pose[i].position.y, pose[i].position.z, \
                                            pose[i].rpy.R, pose[i].rpy.P, pose[i].rpy.Y);
        p.world.collision_objects[i].primitive_poses.push_back(objPose);
        p.object_colors[i] = setColor(id[i], color[i].r, color[i].g, color[i].b, color[i].a);
        p.world.collision_objects[i].operation = p.world.collision_objects[i].ADD;
    }
    p.is_diff = true;
    pub_scene_diff.publish(p);
    ros::Duration(0.5).sleep();
    return checkPubStatus(id);
}

bool PlanningSceneBuilder::checkPubStatus(vector<string> id)
{
    map<string, moveit_msgs::CollisionObject> object;
    object = planning_scene_interface.getObjects(id);
    if(object.size() != id.size())
        return false;
    return true;
}

bool PlanningSceneBuilder::removeObject(std::string& id)
{
    moveit_msgs::PlanningScene p;
    p.world.collision_objects.resize(1);
    p.world.collision_objects[0].id = id;
    p.world.collision_objects[0].operation = p.world.collision_objects[0].REMOVE;
    p.is_diff = true;
    pub_scene_diff.publish(p);
    ros::Duration(0.50).sleep();
    vector<string> idList;
    idList.push_back(id);
    return !checkPubStatus(idList);
}





