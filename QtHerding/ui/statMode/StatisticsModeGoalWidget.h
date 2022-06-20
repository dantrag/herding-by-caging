#ifndef STATISTICSMODEGOALWIDGET_H
#define STATISTICSMODEGOALWIDGET_H

#include "ui_StatisticsModeGoalWidget.h"

namespace Ui
{
  class StatisticsModeGoalWidget;
}

class StatisticsModeGoalWidget : public QWidget, public Ui::StatisticsModeGoalWidget
{
  Q_OBJECT

public:
  explicit StatisticsModeGoalWidget(QWidget *parent = 0);
  ~StatisticsModeGoalWidget();

  std::vector<double> getNextGoal();
  int getNextID() { return nextGoal; }
  void goToNextGoal();
  void reset();

private:
  int nextGoal;
};

#endif // STATISTICSMODEGOALWIDGET_H
