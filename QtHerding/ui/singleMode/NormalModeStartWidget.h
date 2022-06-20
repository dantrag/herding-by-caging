#ifndef NORMALMODESTARTWIDGET_H
#define NORMALMODESTARTWIDGET_H

#include "ui_NormalModeStartWidget.h"

namespace Ui
{
  class NormalModeStartWidget;
}

class NormalModeStartWidget : public QWidget, public Ui::NormalModeStartWidget
{
  Q_OBJECT

public:
  explicit NormalModeStartWidget(QWidget *parent = 0);
  ~NormalModeStartWidget();

public:
  std::vector<double> getNextStart();
  int getNextNRobots();
};

#endif // NORMALMODESTARTWIDGET_H
