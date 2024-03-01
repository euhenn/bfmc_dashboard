#include <QApplication>
#include <QIcon>
#include "dashboard_gui.h"


int main(int argc, char *argv[])
{
  //ros::init(argc, argv, "dashboard_gui_node",ros::init_options::AnonymousName);

  QApplication a(argc, argv);

  DashboardGui w;

  // set the window title as the node name
  //w.setWindowTitle(QString::fromStdString(
  //                     ros::this_node::getName()));

  w.setWindowTitle("DEI Dashboard");

  // load the icon from our qrc file and set it as the application icon
  QIcon icon(":/icons/car_icon.png");
  w.setWindowIcon(icon);

  w.show();

  return a.exec();

}

