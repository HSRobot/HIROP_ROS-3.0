#include <iostream>

#include "ros/ros.h"

#include "perepionFun.h"
#include <assert.h>
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "perception_bridge");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    pereptionFun pece(n);
    string path = ros::package::getPath("perception_bridge");
    path += "/config/filter.yaml";
    pece.loadLocalParam(path);
    pece.printInfo();
//    for(int i = 0; i< 1000;i++){
//        pece.loadPCD("/home/fshs/work/hirop/test/cloud_0000.pcd");
//        std::cout << " loadPCD ok" <<std::endl;

//        pece.process();
//        std::cout << " process ok" <<std::endl;

//    }
//    pece.savePCD();
//    std::cout << " exit ok" <<std::endl;

    pece.start();

}
