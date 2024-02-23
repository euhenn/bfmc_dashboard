#ifndef DASHBOARD_GUI_H
#define DASHBOARD_GUI_H

#include <QWidget>

namespace Ui {
class DashboardGui;
}

class DashboardGui : public QWidget
{
  Q_OBJECT

public:
  explicit DashboardGui(QWidget *parent = nullptr);
  ~DashboardGui();

private:
  Ui::DashboardGui *ui;
};

#endif // DASHBOARD_GUI_H
