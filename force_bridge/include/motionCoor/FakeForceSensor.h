#pragma once
#include "vector"
#include "iostream"
#include "string"
#include <math.h>
#include <array>
#include <queue>
using namespace std ;
class FakeForceSensor
{
public:
    explicit FakeForceSensor(const string & path= "Fake");
    int  init();

    int getForceVal(vector<double> &data);

    string getSensorInfo();

    void setTimeLength(double endTime);


    void pushAxisData(const std::array<double, 6>& element );

    void resetAxisData();

    bool popAxisData(std::array<double, 6>& element);

    void setForceEnable(const vector<bool> & data);
private:
    double drawSin(double &x, const double & piValue);
    double drawCos(double &x, const double &piValue);

private:
    int fakeTime;
    double startTime, endTime;
    string name;
    double timeStamp = 0.04;
    double piValue = M_PI;
    double gain = 2;
    double tempValue;
    std::array<bool, 6> sensorEnable;
    std::queue<std::array<double, 6>> axisQueue;

};

//当前的采集周期是i 10 / 5000

