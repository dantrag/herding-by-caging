#ifndef STATISTICSMODERANDOMSTARTWIDGET_H
#define STATISTICSMODERANDOMSTARTWIDGET_H

#include "ui_StatisticsModeRandomStartWidget.h"

class StatisticsModeRandomStartWidget : public QWidget,
                                        public Ui::StatisticsModeRandomStartWidget
{
  Q_OBJECT

public:
  explicit StatisticsModeRandomStartWidget(QWidget *parent = 0);
  ~StatisticsModeRandomStartWidget();
};

#endif // STATISTICSMODERANDOMSTARTWIDGET_H
