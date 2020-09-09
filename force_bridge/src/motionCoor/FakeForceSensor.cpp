#include <FakeForceSensor.h>

FakeForceSensor::FakeForceSensor(const string & path):name(path){
    init();
}
int  FakeForceSensor::init()
{
    sensorEnable = {false,false,true,false, false,false};

    startTime = 0;
    fakeTime = 0;
    return 0;
}

int FakeForceSensor::getForceVal(vector<double> &data)
{
    data.clear();
    data.resize(6);
    std::fill(data.begin(), data.begin() + data.size(), 0);

    //获取时间基准数值
    double bias = tempValue - this->endTime;
    if(fabs(bias) > 0 &&  fabs(bias) < 0.04 )
        return -1;

    tempValue += timeStamp;
    //遍历数组
    for(int i = 0; i< sensorEnable.size(); i++)
    {
        if(!sensorEnable[i])
            continue;
        if(i == 5)
        {
            data[i] = drawCos(tempValue,piValue);

        }
        else
            data[i] = drawSin(tempValue,piValue )*0.08;
    }
    return 0;
}


string FakeForceSensor::getSensorInfo()
{
    return name;
}

void FakeForceSensor::setTimeLength(double endTime)
{
    this->endTime = endTime;
    tempValue = 0;
}


void FakeForceSensor::pushAxisData(const std::array<double, 6>& element )
{
    axisQueue.push(element);

}

void FakeForceSensor::resetAxisData(){
    std::queue<std::array<double, 6>> temp;
    axisQueue.swap(temp);
}

bool FakeForceSensor::popAxisData(std::array<double, 6>& element)
{
    element = axisQueue.front();
    axisQueue.pop();
}

void FakeForceSensor::setForceEnable(const vector<bool> &data)
{
    for(int i = 0;i < 6 ; i++)
    {
        sensorEnable[i] = data[i];
    }
}

double FakeForceSensor::drawSin(double &x, const double & piValue)
{
    double temp = (x/endTime);
    return gain *sin(temp*piValue);
}

double FakeForceSensor::drawCos(double &x,const double & piValue)
{
    double temp = (x/endTime);

    return gain *cos(temp*piValue);
}
