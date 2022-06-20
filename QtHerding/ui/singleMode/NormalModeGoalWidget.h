#ifndef NORMALMODEGOALWIDGET_H
#define NORMALMODEGOALWIDGET_H

#include "ui_NormalModeGoalWidget.h"

namespace Ui
{
  class NormalModeGoalWidget;
}

class NormalModeGoalWidget : public QWidget, public Ui::NormalModeGoalWidget
{
  Q_OBJECT

public:
  explicit NormalModeGoalWidget(QWidget *parent = 0);
  ~NormalModeGoalWidget();

public:
  std::vector<double> getNextGoal();
};

#endif // NORMALMODEGOALWIDGET_H
