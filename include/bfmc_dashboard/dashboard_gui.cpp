#include "dashboard_gui.h"
#include "ui_dashboard_gui.h"

DashboardGui::DashboardGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::DashboardGui)
{
  ui->setupUi(this);
}

DashboardGui::~DashboardGui()
{
  delete ui;
}
