#ifndef STATISTICSMODESTARTWIDGET_H
#define STATISTICSMODESTARTWIDGET_H

#include "ui_StatisticsModeStartWidget.h"
#include "StatisticsModeCustomStartWidget.h"
#include "StatisticsModeRandomStartWidget.h"

#include <memory>

namespace Ui
{
  class StatisticsModeStartWidget;
}

class StatisticsModeStartWidget : public QWidget, public Ui::StatisticsModeStartWidget
{
  Q_OBJECT

public:
  explicit StatisticsModeStartWidget(QWidget *parent = 0);
  ~StatisticsModeStartWidget();

  std::vector<double> getNextStart();
  int getNextNRobots();
  int getNextID() { return nextStart; }

  void goToNextStart();
  void reset();

public:
  std::shared_ptr<StatisticsModeCustomStartWidget> customWidget;
  std::shared_ptr<StatisticsModeRandomStartWidget> randomWidget;

  int nextStart;
  std::vector<double> cachedRandomStart;
};

#endif // STATISTICSMODESTARTWIDGET_H
