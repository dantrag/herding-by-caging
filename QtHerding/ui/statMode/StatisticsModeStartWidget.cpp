#include "StatisticsModeStartWidget.h"
#include "ompl/util/RandomNumbers.h"
#include "backend/ashapes/ashapes2d.h"
#include <QTextBlock>
#include <QString>
#include <sstream>

StatisticsModeStartWidget::StatisticsModeStartWidget(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  customWidget.reset(new StatisticsModeCustomStartWidget());
  randomWidget.reset(new StatisticsModeRandomStartWidget());
  toolBox->removeItem(0);
  toolBox->insertItem(0, randomWidget.get(), QString("Generate random"));
  toolBox->insertItem(1, customWidget.get(), QString("Use pre-defined"));
  connect(randomWidget->normalizeCheckBox, SIGNAL(clicked(bool)), randomWidget->normalDSpinBox, SLOT(setEnabled(bool)));
  reset();
}

StatisticsModeStartWidget::~StatisticsModeStartWidget()
{
}

std::vector<double> StatisticsModeStartWidget::getNextStart()
{
  std::vector<double> coords;
  if (toolBox->currentWidget() == randomWidget.get())
  {
    // Random start generation mode
    if (nextStart >= randomWidget->nStartsSpinBox->value())
      return {};

    if (cachedRandomStart.size() == 0)
    {
      ompl::RNG rng;
      double minX = fmin(randomWidget->goalX1SpinBox->value(), randomWidget->goalX2SpinBox->value());
      double maxX = fmax(randomWidget->goalX1SpinBox->value(), randomWidget->goalX2SpinBox->value());
      double minY = fmin(randomWidget->goalY1SpinBox->value(), randomWidget->goalY2SpinBox->value());
      double maxY = fmax(randomWidget->goalY1SpinBox->value(), randomWidget->goalY2SpinBox->value());
      double normD = randomWidget->normalDSpinBox->value();
      int n = randomWidget->nRobotsSpinBox->value();
      int tries = 0;
      double eps, maxEdge;
      int cycle;
      double D = 0;
      while (tries < 100)
      {
        coords.clear();
        for (int i = 0; i < n; ++i)
        {
          double X = rng.uniformReal(minX, maxX);
          double Y = rng.uniformReal(minY, maxY);
          coords.push_back(X);
          coords.push_back(Y);
        }
        D = 0;
        for (int i = 0; i < n - 1; ++i)
          for (int j = i + 1; j < n; ++j)
          {
            D = fmax(D, sqrt(pow(coords[i * 2] - coords[j * 2], 2) +
                             pow(coords[i * 2 + 1] - coords[j * 2 + 1], 2)));
          }
        std::queue<double> q;
        for (auto c : coords)
        {
          q.push(c);
        }
        // note: eps threshold should still hold after normalizing
        runVerification(q, eps, maxEdge, cycle, 0.3 * D / normD);
        if (eps > 1e-5) break; // means gt 0
        tries++;
      }
      if (tries >= 100)
        return {};

      if (randomWidget->normalizeCheckBox->isChecked())
      {
        if (D < 1e-10) return {};
        double scale = normD / D;
        double centerX = 0., centerY = 0.;
        for (int i = 0; i < n; ++i)
        {
          centerX += coords[i * 2];
          centerY += coords[i * 2 + 1];
        }
        centerX /= n;
        centerY /= n;
        for (int i = 0; i < n; ++i)
          coords[i * 2] = centerX + (coords[i * 2] - centerX) * scale;
        for (int i = 0; i < n; ++i)
          coords[i * 2 + 1] = centerY + (coords[i * 2 + 1] - centerY) * scale;
      }
      for (auto c : coords)
      {
        cachedRandomStart.push_back(c);
      }
    }
    else
    {
      for (auto c : cachedRandomStart)
      {
        coords.push_back(c);
      }
    }
  }
  else
  if (nextStart < customWidget->startsEdit->document()->lineCount())
  {
    double x, y;
    std::stringstream ss(customWidget->startsEdit->document()->findBlockByLineNumber(nextStart).text().toStdString());
    while (ss >> x >> y)
    {
      coords.push_back(x);
      coords.push_back(y);
    }
  }
  return coords;
}

int StatisticsModeStartWidget::getNextNRobots()
{
  return getNextStart().size() / 2;
}

void StatisticsModeStartWidget::goToNextStart()
{
  nextStart++;
  cachedRandomStart.clear();
}

void StatisticsModeStartWidget::reset()
{
  nextStart = 0;
  cachedRandomStart.clear();
}
