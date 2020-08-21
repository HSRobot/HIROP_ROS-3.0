#include <hirop/datamanager/jointdata.h>
#include <vector>
#include <hirop/datamanager/file_datawriter.h>

#include <ros/ros.h>
#include <hirop_msgs/GetData.h>
#include <hirop_msgs/SaveData.h>

#include <hirop_msgs/saveJointData.h>
#include <hirop_msgs/loadJointsData.h>
#include <hirop_msgs/savePoseData.h>
#include <hirop_msgs/loadPoseData.h>
#include <hirop_msgs/saveDataEnd.h>
#include  <hirop_msgs/setRobotType.h>

using namespace hirop::data_manager;

class DMBridge{

public:
    DMBridge(){
        // FileDataWriter fileDateWrite;
        // writer = &fileDateWrite;
        getDataSrv = _n.advertiseService("get_data", &DMBridge::getCallback, this);
        saveDataSrv = _n.advertiseService("save_data", &DMBridge::saveCallback, this);
        saveJointDataMultiSrv = _n.advertiseService("add_joint_data", &DMBridge::SaveMultiCallback, this);
        loadJointDataMultiSrv = _n.advertiseService("load_joint_data", &DMBridge::loadDataMultiCallback, this);
        savePoseDataMultiSrv = _n.advertiseService("add_pose_data", &DMBridge::savePoseMulti, this);
        loadPoseDataMultiSrv = _n.advertiseService("load_pose_data", &DMBridge::loadPoseMulti, this);
        saveDataMultiEnd = _n.advertiseService("save_data_end", &DMBridge::saveMultiEndCallBack, this);
        setRobotType = _n.advertiseService("set_robot_type", &DMBridge::setRobotTypeCB, this);
    }

    bool getCallback(hirop_msgs::GetData::Request &req, hirop_msgs::GetData::Response &res){

        DataUri uri(req.name);

        uri.setUriFromStr(req.uri);

        std::string rawData = writer.loadRawData(uri);
        res.data =  rawData;

        return true;
    }

    bool saveCallback(hirop_msgs::SaveData::Request &req, hirop_msgs::SaveData::Response &res){

        DataUri uri(req.name);

        uri.setUriFromStr(req.uri);

        std::string rawData = req.data;

        writer.saveRawData(rawData, uri);

        return true;
    }

    bool setRobotTypeCB(hirop_msgs::setRobotType::Request& req, hirop_msgs::setRobotType::Response& rep)
    {
        writer.setConfing(req.robot_name, req.DOF);
        return true;
    }

    bool SaveMultiCallback(hirop_msgs::saveJointData::Request &req, hirop_msgs::saveJointData::Response& rep)
    {
        std::vector<double> joint  = req.joint;
        ROS_INFO_STREAM(joint[0] << " "  << joint[1] << " "  << joint[2] << " "  << joint[3] << " "  << joint[4] << " "  << joint[5]);
        if(writer.saveJointDataMulti(joint) == 0)
        {
            ROS_INFO("save multi SUCCESS");
            rep.result = 0;
            return true;
        }
        ROS_INFO("save multi FAILED");
        return false;
    }

    bool saveMultiEndCallBack(hirop_msgs::saveDataEnd::Request &req, hirop_msgs::saveDataEnd::Response& rep)
    {
        DataUri uri(req.name);
        uri.setUriFromStr(req.uri);
        if(writer.saveDataMultiEnd(uri) == 0)
        {
            rep.result = 0;
            return true;
        }
        rep.result = -1;
        return false;
    }

    bool loadDataMultiCallback(hirop_msgs::loadJointsData::Request & req, hirop_msgs::loadJointsData::Response& rep)
    {
        DataUri uri(req.name);
        uri.setUriFromStr(req.uri);
        std::vector<std::vector<double> > joints = writer.loadJointDataMulti(uri);
        rep.joints.resize(joints.size());
        for(int i=0; i < joints.size(); ++i)
        {
            for(int j=0; j < joints[0].size(); j++)
            {
                rep.joints[i].joint.push_back(joints[i][j]);
            }
        }
        return true;
    }

    bool savePoseMulti(hirop_msgs::savePoseData::Request & req, hirop_msgs::savePoseData::Response& rep)
    {
        req.pose.
        if(writer.savePoseDataMulti(req.pose) == 0)
        {
            ROS_INFO("save pose SUCCESS");
            rep.result = 0;
            return true;
        }
        ROS_INFO("save pose FAILED");
        return false;
    }

    bool loadPoseMulti(hirop_msgs::loadPoseData::Request & req, hirop_msgs::loadPoseData::Response& rep)
    {
        DataUri uri(req.name);
        uri.setUriFromStr(req.uri);
        rep.poses = writer.loadPoseDataMulti(uri);
        return true;
    }

private:
    ros::ServiceServer getDataSrv;
    ros::ServiceServer saveDataSrv;
    ros::ServiceServer saveJointDataMultiSrv;
    ros::ServiceServer loadJointDataMultiSrv;
    ros::ServiceServer savePoseDataMultiSrv;
    ros::ServiceServer loadPoseDataMultiSrv;
    ros::ServiceServer setRobotType;

    ros::ServiceServer saveDataMultiEnd;
    ros::NodeHandle _n;
    FileDataWriter writer;

};

int main(int argc, char** argv){

    ros::init(argc, argv, "dm_bridge");

    DMBridge bridge;

    ros::spin();

}
