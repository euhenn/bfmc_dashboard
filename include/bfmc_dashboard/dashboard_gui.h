#ifndef DASHBOARD_GUI_H
#define DASHBOARD_GUI_H

#include <QDebug>
#include <QWidget>
#include <QProcess>
#include <qtimer.h>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
//#include <bfmc_dashboard/vehicles.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

namespace Ui {
class DashboardGui;
}

class DashboardGui : public QWidget
{
  Q_OBJECT

public:
  explicit DashboardGui(QWidget *parent = nullptr);
  ~DashboardGui();

  void leftsonarCallback(const std_msgs::Float32::ConstPtr& msg);
  void centersonarCallback(const std_msgs::Float32::ConstPtr& msg);
  void rightsonarCallback(const std_msgs::Float32::ConstPtr& msg);
  void speedCallback(const std_msgs::Float32::ConstPtr& msg);
  void steerCallback(const std_msgs::Float32::ConstPtr& msg);
  void positionCallback(const std_msgs::Float32::ConstPtr& msg);
  void dollCallback(const std_msgs::Float32::ConstPtr& msg);
  void carCallback(const std_msgs::Float32::ConstPtr& msg);
  //void localizationCallback(const bfmc_dashboard::vehicles::ConstPtr& msg);


public slots:
  void spinOnce();
  //void updateImage(const sensor_msgs::ImageConstPtr& msg);

private slots:
  void initializeROS();
  void on_stopbutton_clicked();
  void on_startbutton_clicked();
  void on_startROS_clicked();
  void on_resetROS_clicked();
  void on_mainbrainbutton_clicked();
  void on_speeeedbutton_clicked();

private:
  Ui::DashboardGui *ui;
  QTimer *ros_timer;

  ros::NodeHandlePtr nh_;
  ros::Subscriber leftsonar_sub_;
  ros::Subscriber centersonar_sub_;
  ros::Subscriber rightsonar_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber steer_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber car_sub_;
  ros::Subscriber doll_sub_;
  //ros::Subscriber localization_sub_;

  image_transport::Subscriber image_sub;
  image_transport::ImageTransport *it;

};

#endif // DASHBOARD_GUI_H

