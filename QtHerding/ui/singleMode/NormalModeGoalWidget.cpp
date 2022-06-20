#include "NormalModeGoalWidget.h"

NormalModeGoalWidget::NormalModeGoalWidget(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
}

NormalModeGoalWidget::~NormalModeGoalWidget()
{
}

std::vector<double> NormalModeGoalWidget::getNextGoal()
{
  return {
    goalX1SpinBox->value(), goalY1SpinBox->value(),
    goalX2SpinBox->value(), goalY2SpinBox->value()
  };
}
