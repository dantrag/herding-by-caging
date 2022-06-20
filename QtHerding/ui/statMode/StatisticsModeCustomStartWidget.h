#ifndef STATISTICSMODECUSTOMSTARTWIDGET_H
#define STATISTICSMODECUSTOMSTARTWIDGET_H

#include "ui_StatisticsModeCustomStartWidget.h"

class StatisticsModeCustomStartWidget : public QWidget,
                                        public Ui::StatisticsModeCustomStartWidget
{
  Q_OBJECT

public:
  explicit StatisticsModeCustomStartWidget(QWidget *parent = 0);
  ~StatisticsModeCustomStartWidget();
};

#endif // STATISTICSMODECUSTOMSTARTWIDGET_H
