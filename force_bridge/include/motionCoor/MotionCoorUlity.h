#pragma once


#include <interPolTool.h>
#include <boost/shared_ptr.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <queue>
using namespace tk;

struct ImpandenceJointData
{
    ImpandenceJointData() {

    }
    std::queue<std::array<double, 6>> jointQue;

    friend  ImpandenceJointData& operator<< (  ImpandenceJointData& ptr,std::array<double, 6>& arr)
    {
        ptr.jointQue.push(arr);
        return ptr;
    }

    friend std::vector<double>& operator<< ( std::vector<double>& vec, ImpandenceJointData& ptr)
    {

        if( ptr.jointQue.size() > 0){
            std::array<double, 6>  arr = ptr.jointQue.front();

            vec.resize(6);
            std::copy(arr.begin(), arr.end()-1, vec.begin());
            ptr.jointQue.pop();
        }
        return vec;
    }

    bool getElemt( std::vector<double>& vec){
        if( jointQue.size() > 0){
            std::array<double, 6>  arr = jointQue.front();

            vec.resize(6);
            std::copy(arr.begin(), arr.end()-1, vec.begin());
            jointQue.pop();
            return true;

        }
        return false;
    }

    void clear()
    {
        jointQue = std::queue<std::array<double, 6>>();
    }


    int size(){
        return jointQue.size();
    }
};

class MotionCoorUlity
{
public:
    MotionCoorUlity();
    /**
     * @brief setTraject 监听 到直线的路径 或者是其他的重规划轨迹
     * @param traj
     */
    void setTraject(const trajectory_msgs::JointTrajectory & traj);

    /**
     * @brief computeTraj
     * @return
     */
    bool computeTraj();

    /**
     * @brief getComputTraj
     * @return
     */
    bool getComputTraj(std::vector<double> &out);

    /**
     * @brief getSize
     * @return
     */
    int getSize();
private:
    void timeCompute();
private:
    std::vector<double> oldTimeSeq,timeSeq;
    double timeStamp;
    std::vector<boost::shared_ptr<InterPolTool>> toolGroup;
    boost::shared_ptr<ImpandenceJointData> jointDataQue;
    trajectory_msgs::JointTrajectory curMsg;
    std::map<int, std::vector<double>> jointDataGroup;
};
