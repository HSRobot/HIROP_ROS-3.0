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
    taskInputCmd = nh.serviceClient<hirop_msgs::taskInputCmd>("/TaskServerCmd");
    taskRet = nh.subscribe("taskRet", 1, &MainWindow::taskRetCb,this);
}

void MainWindow::taskRetCb(const hirop_msgs::taskCmdRet &ret)
{
    std::string msg = ret.message;
    std::cout << "callback resut :"<< ret.ret<<std::endl;
    ui->plainTextEdit->setPlainText(QString::fromStdString(msg));
}

void MainWindow::on_pushButton_clicked()
{
    hirop_msgs::taskInputCmd cmd;
    cmd.request.param = std::vector<std::string>{"test..."};
    cmd.request.behavior = "running";
    taskInputCmd.call(cmd);
    ROS_INFO_STREAM("RUNNING ");
}

void MainWindow::on_pushButton_2_clicked()
{
    hirop_msgs::taskInputCmd cmd;
    cmd.request.param = std::vector<std::string>{"test..."};
    cmd.request.behavior = "stopping";
    taskInputCmd.call(cmd);
}
