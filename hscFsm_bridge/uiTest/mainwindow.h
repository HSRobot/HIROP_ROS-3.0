#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

#include <ros/ros.h>
#include <hirop_msgs/taskCmdRet.h>
#include <hirop_msgs/taskInputCmd.h>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setRosHandler(ros::NodeHandle & nh);

    void initRosParam();

private:
    void taskRetCb(const hirop_msgs::taskCmdRet & ret);
private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();

private:
    ros::ServiceClient taskInputCmd;
    ros::Subscriber taskRet;
private:
    Ui::MainWindow *ui;
    ros::NodeHandle nh;
};

#endif // MAINWINDOW_H
