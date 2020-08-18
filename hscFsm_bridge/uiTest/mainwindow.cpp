#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <ros/ros.h>

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

void MainWindow::on_pushButton_clicked()
{
    ROS_INFO_STREAM("RUNNING ");
}
