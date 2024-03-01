#include "dashboard_gui.h"
#include "ui_dashboard_gui.h"

DashboardGui::DashboardGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::DashboardGui)
{
  ui->setupUi(this);
  // setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

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
  //else
      //QApplication::quit();
}

void DashboardGui::chatterCallback(const std_msgs::String::ConstPtr &msg){
  auto qstring_msg = QString::fromStdString( msg->data.c_str() );
  ui->chatter->setText(qstring_msg);
}

void DashboardGui::temperatureCallback(const std_msgs::Float32::ConstPtr& msg)
{
  float temperature = msg->data;
  // Convert temperature to string for display
  QString temp_str = QString::number(static_cast<double>(temperature), 'f', 2); // Assuming you want to display temperature with 2 decimal places
  ui->temp->setText(temp_str); // Update the QLabel with the temperature value
}

void DashboardGui::humidityCallback(const std_msgs::Float32::ConstPtr& msg)
{
  float humidity = msg->data;
  // Convert temperature to string for display
  QString hum_str = QString::number(static_cast<double>(humidity), 'f', 2);
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

void DashboardGui::updateImage(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
      // ROS image message to OpenCV format
      cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

      // OpenCV to QImage, BGR to RGB conversion
      cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
      QImage qimage(frame.data, frame.cols, frame.rows, static_cast<int>(frame.step), QImage::Format_RGB888);

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
  const char* ssh_stop_command = "ssh pi@192.168.206.104 'pkill -f roslaunch && killall -9 rosmaster && killall -9 rosout'";

  system(ssh_stop_command);
}

void DashboardGui::on_startbutton_clicked()
{
  // Command to SSH and execute the script on Raspberry Pi
  const char* ssh_start_command = "ssh pi@192.168.206.104 'cd ~/Desktop/eugen_ws && source devel/setup.bash && export ROS_MASTER_URI='http://192.168.206.72:11311' && rosrun package_camera cameraNode_640x480.py'";
  // Execute the SSH com mand
  system(ssh_start_command);
}


void DashboardGui::on_startROS_clicked()
{
  initializeROS();

}

void DashboardGui::initializeROS()
{
  if (ros::master::getURI().empty())
  {
      std::string master_uri = "http://192.168.1.60:11311";
      setenv("ROS_MASTER_URI", master_uri.c_str(), 1);
  }

  if (!ros::isInitialized())
  {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "dashboard_gui_node", ros::init_options::AnonymousName);
      ui->startROS->setStyleSheet("background-color: green;");
  }

    // connect ROS callbacks
    nh_ = ros::NodeHandlePtr(new ros::NodeHandle);
    image_transport::ImageTransport it(*nh_);
    chatter_sub_ = nh_->subscribe("chatter", 1, &DashboardGui::chatterCallback, this);
    temp_sub_ = nh_->subscribe("temperature", 1, &DashboardGui::temperatureCallback, this);
    humidity_sub_ = nh_->subscribe("humidity", 1, &DashboardGui::humidityCallback, this);
    hello_pub_ = nh_->advertise<std_msgs::String>("hello", 10);
    led_pub_ = nh_->advertise<std_msgs::Bool>("led_status", 10);
    our_pub_ = nh_->advertise<std_msgs::Empty>("our_topic", 10);
    image_sub = it.subscribe("/automobile/camera_image", 1, &DashboardGui::updateImage, this);
    ui->startROS->setStyleSheet("background-color: blue;");
}
