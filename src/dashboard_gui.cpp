#include "dashboard_gui.h"
#include "ui_dashboard_gui.h"



DashboardGui::DashboardGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::DashboardGui),
  nh_(new ros::NodeHandle("~")),
  it(*nh_)
{
  ui->setupUi(this);

  //nh_.reset(new ros::NodeHandle("~")); without smart pointer reset

  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate


  // setup subscriber by according to the ~/chatter_topic param
  std::string listen_topic;
  nh_->param<std::string>("listen_topic",listen_topic,"/talker/chatter");
  chatter_sub_ = nh_->subscribe<std_msgs::String>(listen_topic, 1, &DashboardGui::chatterCallback, this);

  // publish a message on the channel specified by ~/hello_topic param
  std::string hello_topic;
  nh_->param<std::string>("hello_topic",hello_topic,"chatter");
  hello_pub_ = nh_->advertise<std_msgs::String>(hello_topic,1);


  // subscriber to image
  image_sub = it.subscribe("/camera_image", 1, &DashboardGui::updateImage, this);

  std::string led_topic;
  nh_->param<std::string>("led_topic", led_topic, "/toggle_led");
  led_pub_ = nh_->advertise<std_msgs::Empty>(led_topic, 1);

  std::string temp_topic;
  nh_->param<std::string>("temp_topic",temp_topic,"/automobile/sonar/ahead/center");
  temp_sub_ = nh_->subscribe<std_msgs::Float32>(temp_topic, 1, &DashboardGui::temperatureCallback, this);

  std::string humidity_topic;
  nh_->param<std::string>("humidity_topic",humidity_topic,"/humidity");
  humidity_sub_ = nh_->subscribe<std_msgs::Float32>(humidity_topic, 1, &DashboardGui::humidityCallback, this);

  std::string our_test;
  nh_->param<std::string>("our_test", our_test, "/topic_chatter");
  our_pub_ = nh_->advertise<std_msgs::String>(our_test, 1);
}

DashboardGui::~DashboardGui()
{
  delete ui;
  delete ros_timer;
}

void DashboardGui::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

void DashboardGui::chatterCallback(const std_msgs::String::ConstPtr &msg){
  auto qstring_msg = QString::fromStdString( msg->data.c_str() );
  ui->chatter->setText(qstring_msg);
}

void DashboardGui::temperatureCallback(const std_msgs::Float32::ConstPtr& msg)
{
  float temperature = msg->data;
  // Convert temperature to string for display
  QString temp_str = QString::number(temperature, 'f', 2); // Assuming you want to display temperature with 2 decimal places
  ui->temp->setText(temp_str); // Update the QLabel with the temperature value
}

void DashboardGui::humidityCallback(const std_msgs::Float32::ConstPtr& msg)
{
  float humidity = msg->data;
  // Convert temperature to string for display
  QString hum_str = QString::number(humidity, 'f', 2);
  ui->humidity->setText(hum_str);
}

void DashboardGui::on_hi_button_clicked()
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "hello world " << ui->hi_num->value();
  msg.data = ss.str();

  hello_pub_.publish(msg);

  ui->hi_num->setValue(ui->hi_num->value()+1);
}

/*void DashboardGui::updateCamera()
{
  cv::Mat frame;
  capture >> frame;
  // Some images won't show properly without .step, don t mind the warning :)
  QImage qimage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(qimage.rgbSwapped());
  QPixmap scaledPixmap = pixmap.scaled(ui->image_feed->size(), Qt::KeepAspectRatio);
  ui->image_feed->setPixmap(scaledPixmap);
}
*/

void DashboardGui::updateImage(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
      // ROS image message to OpenCV format
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

      // OpenCV to QImage, BGR to RGB conversion
      cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
      QImage qimage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);

      // display the image in QLabel
      QPixmap pixmap = QPixmap::fromImage(qimage);
      QPixmap scaledPixmap = pixmap.scaled(ui->image_feed->size(), Qt::KeepAspectRatio);
      ui->image_feed->setPixmap(scaledPixmap);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
  }

}

void DashboardGui::on_led_button_clicked()
{
  std_msgs::Empty msg; // Create an empty message of type std_msgs::Empty
  led_pub_.publish(msg);
}

void DashboardGui::on_stopbutton_clicked()
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "STOP";
  msg.data = ss.str();

  our_pub_.publish(msg);
}

void DashboardGui::on_startbutton_clicked()
{
  // Command to SSH and execute the script on Raspberry Pi
  const char* ssh_command = "ssh pi@192.168.206.104 'cd ~/Desktop/eugen_ws && source devel/setup.bash && source network_conf.bash && roslaunch package_camera camera_test.launch'";
  // Execute the SSH command
  system(ssh_command);
}
