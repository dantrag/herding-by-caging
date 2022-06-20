#include "StatisticsModeWidget.h"

StatisticsModeWidget::StatisticsModeWidget(QWidget *parent) : InputWidget(parent)
{
  setupUi(this);
  startWidget.reset(new StatisticsModeStartWidget(this));
  goalWidget.reset(new StatisticsModeGoalWidget(this));
  startBox->layout()->addWidget(startWidget.get());
  goalBox->layout()->addWidget(goalWidget.get());
}

StatisticsModeWidget::~StatisticsModeWidget()
{
}

std::vector<double> StatisticsModeWidget::getNextStart()
{
  return startWidget->getNextStart();
}

int StatisticsModeWidget::getNextNRobots()
{
  return startWidget->getNextNRobots();
}

std::vector<double> StatisticsModeWidget::getNextGoal()
{
  return goalWidget->getNextGoal();
}

int StatisticsModeWidget::getNextStartID()
{
  return startWidget->getNextID();
}

int StatisticsModeWidget::getNextGoalID()
{
  return goalWidget->getNextID();
}

void StatisticsModeWidget::goToNextEntry()
{
  goalWidget->goToNextGoal();
  if (goalWidget->getNextGoal().size() == 0)
  {
    goalWidget->reset();
    startWidget->goToNextStart();
  }
}

void StatisticsModeWidget::reset()
{
  startWidget->reset();
  goalWidget->reset();
}

void StatisticsModeWidget::addStart(QString startCoords)
{
  startWidget->customWidget->startsEdit->insertPlainText(startCoords + "\n");
}

void StatisticsModeWidget::setStarts(QStringList starts_strings)
{
  startWidget->customWidget->startsEdit->clear();
  QString start_string;
  for (auto string : starts_strings)
  {
    start_string += string + " ";
  }
  addStart(start_string);
}
