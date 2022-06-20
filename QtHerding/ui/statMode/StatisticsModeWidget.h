#ifndef STATISTICSMODEWIDGET_H
#define STATISTICSMODEWIDGET_H

#include "ui/InputWidget.h"
#include "StatisticsModeStartWidget.h"
#include "StatisticsModeGoalWidget.h"
#include "ui_StatisticsModeWidget.h"

#include <memory>

namespace Ui
{
  class StatisticsModeWidget;
}

class StatisticsModeWidget : public InputWidget, public Ui::StatisticsModeWidget
{
  Q_OBJECT

public:
  explicit StatisticsModeWidget(QWidget *parent = 0);
  ~StatisticsModeWidget();

public:
  std::vector<double> getNextStart() override;
  std::vector<double> getNextGoal() override;
  int getNextNRobots() override;
  int getNextStartID();
  int getNextGoalID();
  void reset();
  void goToNextEntry();
  void addStart(QString startCoords);
  void setStarts(QStringList starts_strings) override;

private:
  std::shared_ptr<StatisticsModeStartWidget> startWidget;
  std::shared_ptr<StatisticsModeGoalWidget> goalWidget;
};

#endif // STATISTICSMODEWIDGET_H
