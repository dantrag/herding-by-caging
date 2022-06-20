#include "NormalModeWidget.h"

#include <QMessageBox>
#include <QTextBlock>

NormalModeWidget::NormalModeWidget(QWidget *parent) : InputWidget(parent)
{
  setupUi(this);
  startWidget.reset(new NormalModeStartWidget(this));
  goalWidget.reset(new NormalModeGoalWidget(this));
  startBox->layout()->addWidget(startWidget.get());
  goalBox->layout()->addWidget(goalWidget.get());
}

NormalModeWidget::~NormalModeWidget()
{
}

std::vector<double> NormalModeWidget::getNextStart()
{
  return startWidget->getNextStart();
}

int NormalModeWidget::getNextNRobots()
{
  return startWidget->getNextNRobots();
}

std::vector<double> NormalModeWidget::getNextGoal()
{
  return goalWidget->getNextGoal();
}

void NormalModeWidget::setStarts(QStringList starts_strings)
{
  startWidget->startXEdit->clear();
  startWidget->startYEdit->clear();

  QStringList strings;
  for (auto start_string : starts_strings)
  {
    if (start_string != "")
    {
      strings.push_back(start_string);
    }
  }

  for (int i = 0; i < strings.size(); ++i)
  {
    auto start_string = strings[i];

    auto coords = start_string.split(' ');
    if (coords.size() != 2)
    {
      startWidget->startXEdit->clear();
      startWidget->startYEdit->clear();
      QMessageBox msgBox;
      msgBox.setText(QString("Cannot copy coordinates. Wrong number of tokens in a string (given %1, expected 2)").arg(coords.size()));
      msgBox.exec();
      return;
    }
    if (i != strings.size() - 1)
    {
      coords[0] += "\n";
      coords[1] += "\n";
    }
    startWidget->startXEdit->insertPlainText(coords[0]);
    startWidget->startYEdit->insertPlainText(coords[1]);
  }
}
