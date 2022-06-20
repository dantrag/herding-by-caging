#ifndef NORMALMODEWIDGET_H
#define NORMALMODEWIDGET_H

#include "ui/InputWidget.h"
#include "NormalModeStartWidget.h"
#include "NormalModeGoalWidget.h"
#include "ui_NormalModeWidget.h"

#include <memory>

namespace Ui
{
  class NormalModeWidget;
}

class NormalModeWidget : public InputWidget, public Ui::NormalModeWidget
{
  Q_OBJECT

public:
  explicit NormalModeWidget(QWidget *parent = 0);
  ~NormalModeWidget();

public:
  std::vector<double> getNextStart() override;
  std::vector<double> getNextGoal() override;
  virtual int getNextNRobots();

  void setStarts(QStringList starts_strings) override;

private:
  std::shared_ptr<NormalModeStartWidget> startWidget;
  std::shared_ptr<NormalModeGoalWidget> goalWidget;
};

#endif // NORMALMODEWIDGET_H
