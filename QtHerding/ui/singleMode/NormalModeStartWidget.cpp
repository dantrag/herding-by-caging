#include "NormalModeStartWidget.h"
#include <QTextBlock>

NormalModeStartWidget::NormalModeStartWidget(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
}

NormalModeStartWidget::~NormalModeStartWidget()
{
}

std::vector<double> NormalModeStartWidget::getNextStart()
{
  std::vector<double> coords;
  int n = getNextNRobots();
  for (int i = 0; i < n; ++i)
  {
    coords.push_back(startXEdit->document()->findBlockByLineNumber(i).text().toDouble());
    coords.push_back(startYEdit->document()->findBlockByLineNumber(i).text().toDouble());
  }
  return coords;
}

int NormalModeStartWidget::getNextNRobots()
{
  return std::min(startXEdit->document()->lineCount(), startYEdit->document()->lineCount());
}
