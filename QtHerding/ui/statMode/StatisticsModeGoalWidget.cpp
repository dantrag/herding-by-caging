#include "StatisticsModeGoalWidget.h"
#include <QTextBlock>
#include <sstream>

StatisticsModeGoalWidget::StatisticsModeGoalWidget(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  reset();
}

StatisticsModeGoalWidget::~StatisticsModeGoalWidget()
{
}

std::vector<double> StatisticsModeGoalWidget::getNextGoal()
{
  if (nextGoal < goalsEdit->document()->lineCount())
  {
    double x1, y1, x2, y2;
    std::stringstream ss(goalsEdit->document()->findBlockByLineNumber(nextGoal).text().toStdString());
    ss >> x1 >> y1 >> x2 >> y2;
    return {x1, y1, x2, y2};
  }
  return {};
}

void StatisticsModeGoalWidget::goToNextGoal()
{
  nextGoal++;
}

void StatisticsModeGoalWidget::reset()
{
  nextGoal = 0;
}
