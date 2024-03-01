#ifndef DASHBOARD_GUI_H
#define DASHBOARD_GUI_H

#include <QDebug>
#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
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
  void chatterCallback(const std_msgs::String::ConstPtr& msg);
  void temperatureCallback(const std_msgs::Float32::ConstPtr& msg);
  void humidityCallback(const std_msgs::Float32::ConstPtr& msg);

public slots:
  void spinOnce();
  //void updateCamera();
  void updateImage(const sensor_msgs::ImageConstPtr& msg);

private slots:
  void initializeROS();
  void on_hi_button_clicked();
  void on_led_button_clicked();
  void on_stopbutton_clicked();
  void on_startbutton_clicked();
  void on_startROS_clicked();

private:
  Ui::DashboardGui *ui;
  QTimer *ros_timer;

  ros::NodeHandlePtr nh_;
  ros::Subscriber chatter_sub_;
  ros::Subscriber temp_sub_;
  ros::Subscriber humidity_sub_;
  ros::Publisher hello_pub_;
  ros::Publisher  led_pub_;
  ros::Publisher  our_pub_;

  image_transport::Subscriber image_sub;
  image_transport::ImageTransport *it;

};

#endif // DASHBOARD_GUI_H

