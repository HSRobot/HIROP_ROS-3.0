#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setRosHandler(ros::NodeHandle &nh)
{
    this->nh = nh;
    initRosParam();
}

void MainWindow::initRosParam()
{
    nh.serviceClient<hirop_msgs::taskInputCmd>("taskInputCmd");
    nh.subscribe("taskRet", 1, &MainWindow::taskRetCb,this);
}

void MainWindow::taskRetCb(const hirop_msgs::taskCmdRet &ret)
{

}

void MainWindow::on_pushButton_clicked()
{
    ROS_INFO_STREAM("RUNNING ");
}
