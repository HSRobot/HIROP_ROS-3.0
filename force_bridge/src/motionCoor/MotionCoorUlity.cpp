#include <MotionCoorUlity.h>
#include <boost/make_shared.hpp>
#include <array>
//#include <CsvFile.h>

MotionCoorUlity::MotionCoorUlity():timeStamp(0.04) {

    toolGroup.resize(6);
    for(int i = 0; i < 6; i++)
    {
        boost::shared_ptr<InterPolTool> tool = boost::make_shared<InterPolTool>();
        toolGroup[i] = tool;
    }
    jointDataQue = boost::make_shared<ImpandenceJointData>();
}

void MotionCoorUlity::setTraject(const trajectory_msgs::JointTrajectory &traj)
{
    jointDataGroup.clear();
    curMsg = traj;
}

bool MotionCoorUlity::computeTraj()
{
    timeCompute();

    jointDataQue->clear();
    assert(curMsg.points.size() > 0);
    int index = 0;

    //
    int size = curMsg.points.size();

    std::array<double*, 6> array;
    for(int i = 0; i< 6; i++)
    {
        array[i] = new double[size];
    }

    index= 0;
    for(auto it :curMsg.points)
    {
        for(int j= 0; j < 6; j++)
        {
            double val = it.positions.at(j);
            array[j][index] = val;
        }
        index++;
    }

    for(int axis = 0; axis <6;axis++)
    {

        std::vector<double> axisData(size);
        axisData.assign(&array[axis][0], &array[axis][size]);
        toolGroup[axis]->set_points(oldTimeSeq, axisData);
        std::vector<double> data = toolGroup[axis]->getResult(timeSeq);
        jointDataGroup.insert(std::pair<int, std::vector<double>>(axis, data));

    }



    for(int i = 0; i < timeSeq.size(); i++)
    {
        std::array<double, 6> data;
        for(int axis = 0; axis< 6; axis++)
        {
            data.at(axis) = jointDataGroup.at(axis).at(i);
        }
        *jointDataQue<< data;

    }

//    CsvFile::writeFile("/home/fshs/catkin_ws/src/hsr_impedance/tt.csv",\
//                       jointDataGroup);

    for(int i = 0; i< 6; i++)
    {
        delete array.at(i);
    }
    return true;
}



bool MotionCoorUlity::getComputTraj(std::vector<double> &out)
{
    if(jointDataQue->size() <= 0){
        return false;
    }

//    out << *jointDataQue;
//    std::cout << jointDataQue->size()<<std::endl;
    return  jointDataQue->getElemt(out);
}

int MotionCoorUlity::getSize()
{
    return jointDataQue->size();
}

double MotionCoorUlity::getTimeLength()
{
    double endTime = oldTimeSeq.at(oldTimeSeq.size()-1);
    return  endTime;
}

void MotionCoorUlity::timeCompute()
{
    assert(curMsg.points.size() > 0);

    oldTimeSeq.clear();
    for(auto it = curMsg.points.begin(); it != curMsg.points.end(); it++)
    {
        oldTimeSeq.emplace_back(it->time_from_start.toSec());
    }

    auto itBegin = curMsg.points.begin();
    auto itEnd = curMsg.points.rbegin();
    double timelengthDou = itEnd->time_from_start.toSec() - itBegin->time_from_start.toSec();

    timeSeq.clear();
    for(double i = timeStamp; i < timelengthDou; i+=timeStamp)
    {
        timeSeq.emplace_back(i);
    }
}
