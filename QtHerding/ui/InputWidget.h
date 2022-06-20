#ifndef INPUTWIDGET_H
#define INPUTWIDGET_H

#include <QWidget>

class InputWidget : public QWidget
{
  Q_OBJECT

public:
  explicit InputWidget(QWidget *parent = 0);
  ~InputWidget();

public:
  virtual std::vector<double> getNextStart() = 0;
  virtual std::vector<double> getNextGoal() = 0;
  virtual int getNextNRobots() = 0;

  virtual void setStarts(QStringList starts_strings) = 0;
};

#endif // INPUTWIDGET_H
