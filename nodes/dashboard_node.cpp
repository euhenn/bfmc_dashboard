#include <QApplication>
#include <QIcon>
#include "dashboard_gui.h"


int main(int argc, char *argv[])
{

  QApplication a(argc, argv);

  DashboardGui w;

  w.setWindowTitle("DEI Dashboard");

  // load the icon from our .qrc file and set it as the application icon
  QIcon icon(":/icons/car_icon.png");
  w.setWindowIcon(icon);

  w.show();

  return a.exec();

}

