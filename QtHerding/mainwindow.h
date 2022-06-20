#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsItem>
#include <QPixmap>
#include <QModelIndex>

#include "backend/HerdingPlanning.h"
#include "backend/HerdingSampler.h"
#include "planners/RRTNoInterpolation.h"
#include "planners/RRTInterpolation.h"
#include "planners/RRTRandomTranslate.h"

#include "ui/singleMode/NormalModeWidget.h"
#include "ui/statMode/StatisticsModeWidget.h"

namespace Ui {
  class MainWindow;
}

using RRTLocal = RRTInterpolation;
using SE2State = ompl::base::SE2StateSpace::StateType*;
using SE2States = std::vector<SE2State>;

using Coordinate = std::pair<double, double>;
using Coordinates = std::vector<Coordinate>;
using GraphicsItems = std::vector<QGraphicsItem*>;

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

 private slots:
  void init();
  double start();
  void runJob();
  void runStatistics();
  void setMode();
  void showStart(bool bShow);
  void showGoal(bool bShow);
  void showConfig(bool bShow);
  void showPathConfig(bool bShow);
  void showObstacles(bool bShow);
  void showPotentialField(bool bShow);
  void setPotentialFieldOpacity(int percent);
  void drawConfig();
  void drawPathConfig();
  void drawPotentialField();
  void checkStart();
  void changeScale();
  void doubleClickedConf(QModelIndex index);
  void changeEnvironment();
  void copyTableDataToClipboard();
  void savePathToFile();

  // cage acquisition
  void acquireCageLinkTriggered(const QString &link);
  void changeSheepRandomizerMode(int index);
  void randomizeSheepCoordinates();
  void acquireCage();
  void copyAcquiredCageToInput();
  void displayAcquiredCage(std::vector<std::pair<double, double>> &sheep, std::vector<std::pair<double, double>> &robots_start, std::vector<std::pair<double, double>> &robots_final);
  void outputAcquiredCage(std::vector<std::pair<double, double>> &robots_final);
  void saveCageAcquisitionPaths();

 private:
  void loadEnvironment(QString filename);
  void clearItems();
  void initCanvas();
  void drawCanvas();
  void drawObstacles();
  void drawStart();
  void drawGoal();

  Ui::MainWindow *ui;
  std::shared_ptr<HerdingPlanning> planning;
  std::shared_ptr<RRTLocal> rrtplanner;
  std::shared_ptr<NormalModeWidget> singleWidget;
  std::shared_ptr<StatisticsModeWidget> statWidget;
  std::shared_ptr<InputWidget> curWidget;

  unsigned int n;
  SE2States starts;
  SE2States goals;
  std::vector<SE2States> path;
  std::vector<SE2States> configs;
  std::vector<Coordinates> cageAcquisitionPaths;

  QGraphicsItem *envItem, *goalRectItem;
  QGraphicsPixmapItem *fieldItem;
  GraphicsItems startItems, goalItems, configItems, pathItems, obstacleItems;
  GraphicsItems cagePlanSheepItems, cagePlanRobotStartItems, cagePlanRobotFinalItems;
  GraphicsItems cagePlanPathsItems;

  std::string envMesh;
};

#endif // MAINWINDOW_H
