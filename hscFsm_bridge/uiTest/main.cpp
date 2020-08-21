#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ros::init(argc, argv, "uiTest");
    ros::NodeHandle nh;
    ros::AsyncSpinner as(1);
    as.start();
    MainWindow w;
    w.setRosHandler(nh);
    w.show();

    return a.exec();
}
