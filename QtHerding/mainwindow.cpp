#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QTextBlock>
#include <QDesktopWidget>
#include <QBrush>
#include <QTime>
#include <QFile>
#include <QFileDialog>
#include <QStatusBar>
#include <QTextStream>
#include <QMimeData>
#include <QClipboard>
#include "ui_mainwindow.h"

#include "backend/ashapes/ashapes2d.h"

using namespace ompl;
using namespace std;

const std::string robotMesh = "../../QtHerding/meshes/robots/car1_planar_small_robot.dae";
const std::string environmentMesh = "../../QtHerding/meshes/environments/RandomPolygons_really_planar_env.dae";
const double kSearchResolution = M_PIl / 180;
const int kCageAcquisitionMilestonesCount = 5;

enum
{
  Layer_Environment = 0
, Layer_PotentialField
, Layer_Obstacles
, Layer_Start
, Layer_Path
, Layer_Configuration
, Layer_Goal
};

base::StateSamplerPtr allocHerdingSampler(const base::StateSpace *space)
{
  return base::StateSamplerPtr(new HerdingSampler(space));
}

void MainWindow::init()
{
  n = ui->nRobotsSpinBox->value();
  if (n > curWidget->getNextNRobots())
  {
    QMessageBox msgBox;
    msgBox.setText("Too few lines in the input!");
    msgBox.exec();
    return;
  }

  if (planning)
  {
    for (auto conf : configs)
    {
      for (int i = 0; i < n; ++i)
      {
        planning->getSpaceInformation()->getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(i)->as<base::SE2StateSpace>()->freeState(conf[i]);
      }
    }
    for (auto conf : path)
    {
      for (int i = 0; i < n; ++i)
      {
        planning->getSpaceInformation()->getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(i)->as<base::SE2StateSpace>()->freeState(conf[i]);
      }
    }
  }
  planning.reset(new HerdingPlanning(n));

  planning->setRobotMesh(robotMesh);
  planning->setEnvironmentMesh(envMesh);

  base::ScopedState<base::CompoundStateSpace> start(planning->getSpaceInformation());
  base::ScopedState<base::CompoundStateSpace> goal(planning->getSpaceInformation());

  starts.resize(n);
  auto coords = curWidget->getNextStart();
  assert(coords.size() == n * 2);
  for (unsigned int i = 0; i < n; ++i)
  {
    starts[i] = start.get()->as<base::SE2StateSpace::StateType>(i);
    starts[i]->setX(coords[i * 2]);
    starts[i]->setY(coords[i * 2 + 1]);
    starts[i]->setYaw(0.);
  }
  planning->setStartState(start);
  planning->setGoalState(goal);

  rrtplanner.reset(new RRTLocal(planning->getSpaceInformation()));
  rrtplanner->setEpsilonDensity(ui->epsSamplesSpinBox->value());
  rrtplanner->setFractionFar(ui->fractionFarSpinBox->value());
  rrtplanner->setFractionNear(ui->fractionNearSpinBox->value());
  auto goalcoords = curWidget->getNextGoal();
  rrtplanner->setGoalRegionRect(goalcoords[0], goalcoords[1], goalcoords[2], goalcoords[3]);
  rrtplanner->setConfigRegistrator([&](base::State* state/*, base::State* parent*/)
  {
    //
    // This is to display configuration where we came from
    //
    /*
    QString parentStr = "?";
    auto parentState = static_cast<base::CompoundStateSpace::StateType*>(parent);
    for (int i = 0; i < configs.size(); ++i)
    {
      bool bOK = true;
      for (int k = 0; k < n; ++k)
      {
        if ((fabs(configs[i][k]->getX() - parentState->as<base::SE2StateSpace::StateType>(k)->getX()) > 1e-10) ||
            (fabs(configs[i][k]->getY() - parentState->as<base::SE2StateSpace::StateType>(k)->getY()) > 1e-10))
        {
          bOK = false;
          break;
        }
      }
      if (bOK)
      {
        parentStr = QString::number(i + 1);
        break;
      }
    }
    ui->confListWidget->addItem("#" + QString::number(ui->confListWidget->count() + 1)+" <- " + parentStr);
    */
    ui->confListWidget->addItem("#" + QString::number(ui->confListWidget->count() + 1));
    ui->confListWidget->item(ui->confListWidget->count() - 1)->setData(Qt::UserRole, QVariant(ui->confListWidget->count() - 1));
    std::vector<base::SE2StateSpace::StateType*> config(n);
    auto compState = static_cast<base::CompoundStateSpace::StateType*>(state);
    auto newState = static_cast<base::CompoundStateSpace::StateType*>(planning->getSpaceInformation()->allocState());
    for (unsigned int i = 0; i < n; ++i)
    {
      newState->as<base::SE2StateSpace::StateType>(i)->setX(compState->as<base::SE2StateSpace::StateType>(i)->getX());
      newState->as<base::SE2StateSpace::StateType>(i)->setY(compState->as<base::SE2StateSpace::StateType>(i)->getY());
      newState->as<base::SE2StateSpace::StateType>(i)->setYaw(compState->as<base::SE2StateSpace::StateType>(i)->getYaw());
      config[i] = newState->as<base::SE2StateSpace::StateType>(i);
    }
    configs.push_back(config);
  });
  planning->setPlanner(rrtplanner);
  planning->getPlanner()->clear();
  planning->getSpaceInformation()->getStateSpace()->setStateSamplerAllocator(allocHerdingSampler);

  clearItems();
  for (int i = 1; i < n; ++i)
  {
    planning->addRobotMesh(robotMesh);
  }
  planning->setup();
  drawStart();
  drawGoal();
  ui->confListWidget->clear();
  ui->pathListWidget->clear();
  configs.clear();
  path.clear();
}

double MainWindow::start()
{
  init();

  //ui->logEdit->insertPlainText("Running solution at " + QTime::currentTime().toString("hh:mm:ss.zzz")+"...\n");

  base::PlannerStatus solved = planning->solve(double(ui->timeSpinButton->value()));
  if (solved)
  {
    //ui->logEdit->insertPlainText("Found solution at " + QTime::currentTime().toString("hh:mm:ss.zzz")+"!\n");
    //stringstream ss;
    //planning->getSolutionPath().print(ss);
    //ui->logEdit->insertPlainText(QString::fromStdString(ss.str()));
    ui->logEdit->insertPlainText(QString::number(planning->getLastPlanComputationTime()) + "\n");
    auto states = planning->getSolutionPath().getStates();
    path.clear();
    for (auto state : states)
    {
      auto compState = static_cast<base::CompoundStateSpace::StateType*>(state);
      std::vector<ompl::base::SE2StateSpace::StateType*> pathstate(n);
      auto newState = static_cast<base::CompoundStateSpace::StateType*>(planning->getSpaceInformation()->allocState());
      for (unsigned int i = 0; i < n; ++i)
      {
        newState->as<base::SE2StateSpace::StateType>(i)->setX(compState->as<base::SE2StateSpace::StateType>(i)->getX());
        newState->as<base::SE2StateSpace::StateType>(i)->setY(compState->as<base::SE2StateSpace::StateType>(i)->getY());
        newState->as<base::SE2StateSpace::StateType>(i)->setYaw(compState->as<base::SE2StateSpace::StateType>(i)->getYaw());
        pathstate[i] = newState->as<base::SE2StateSpace::StateType>(i);
      }

      path.push_back(pathstate);
    }
    ui->pathListWidget->clear();
    for (int i = 0; i < path.size(); ++i)
    {
      for (int j = 0; j < configs.size(); ++j)
      {
        bool bOK = true;
        for (int k = 0; k < n; ++k)
        {
          if ((fabs(configs[j][k]->getX() - path[i][k]->getX()) > 1e-10) ||
              (fabs(configs[j][k]->getY() - path[i][k]->getY()) > 1e-10))
          {
            bOK = false;
            break;
          }
        }
        if (bOK)
        {
          ui->pathListWidget->addItem("#"+QString::number(i + 1)+" (conf #"+QString::number(j + 1)+")");
          ui->pathListWidget->item(ui->pathListWidget->count() - 1)->setData(Qt::UserRole, QVariant(ui->pathListWidget->count() - 1));
          break;
        }
      }
      if (ui->pathListWidget->count() - 1 != i)
        cout << "Not found path # " << i + 1 << " configuration" << endl;
    }
  }
  else
    ui->logEdit->insertPlainText("-1\n");
  double time = solved ? planning->getLastPlanComputationTime() : -1;
  return time;
}

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  planning = nullptr;
  rrtplanner = nullptr;
  ui->setupUi(this);
  singleWidget.reset(new NormalModeWidget(ui->inputBox));
  statWidget.reset(new StatisticsModeWidget(ui->inputBox));
  dynamic_cast<QVBoxLayout*>(ui->inputBox->layout())->insertWidget(1, singleWidget.get());
  singleWidget->hide();
  dynamic_cast<QVBoxLayout*>(ui->inputBox->layout())->insertWidget(2, statWidget.get());
  statWidget->hide();
  bool bSingle = ui->normalModeButton->isChecked();
  singleWidget->setVisible(bSingle);
  statWidget->setVisible(!bSingle);
  if (bSingle)
  {
    curWidget = singleWidget;
  }
  else
  {
    curWidget = statWidget;
  }

  QDesktopWidget wid;


  int screenWidth = wid.screen()->width();
  int screenHeight = wid.screen()->height();
  this->resize(this->width(), min(1200, screenWidth));
  this->resize(this->height(), min(900, screenHeight));
  int width = this->frameGeometry().width();
  int height = this->frameGeometry().height();
  this->setGeometry((screenWidth/2)-(width/2),(screenHeight/2)-(height/2),width,height);

  ui->nRobotsSpinBox->setValue(curWidget->getNextNRobots());
  loadEnvironment(QString::fromStdString(environmentMesh));

  connect(ui->startButton, SIGNAL(clicked()), this, SLOT(runJob()));
  connect(ui->initButton, SIGNAL(clicked()), this, SLOT(init()));
  connect(ui->showStartCheckBox, SIGNAL(clicked(bool)), this, SLOT(showStart(bool)));
  connect(ui->showGoalCheckBox, SIGNAL(clicked(bool)), this, SLOT(showGoal(bool)));
  connect(ui->showConfCheckBox, SIGNAL(clicked(bool)), this, SLOT(showConfig(bool)));
  connect(ui->showPathConfCheckBox, SIGNAL(clicked(bool)), this, SLOT(showPathConfig(bool)));
  connect(ui->showObstaclesCheckBox, SIGNAL(clicked(bool)), this, SLOT(showObstacles(bool)));
  connect(ui->showFieldCheckBox, SIGNAL(clicked(bool)), this, SLOT(showPotentialField(bool)));
  connect(ui->confListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(drawConfig()));
  connect(ui->pathListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(drawPathConfig()));
  connect(ui->fieldOpacitySlider, SIGNAL(valueChanged(int)), this, SLOT(setPotentialFieldOpacity(int)));
  connect(ui->checkStartButton, SIGNAL(clicked()), this, SLOT(checkStart()));

  connect(ui->modeButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(setMode()));
  connect(ui->scaleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(changeScale()));
  connect(ui->confListWidget, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(doubleClickedConf(QModelIndex)));
  connect(ui->copyStatsButton, SIGNAL(clicked()), this, SLOT(copyTableDataToClipboard()));

  connect(ui->loadEnvButton, SIGNAL(clicked()), this, SLOT(changeEnvironment()));

  connect(ui->savePathToFile, SIGNAL(clicked()), this, SLOT(savePathToFile()));

  // cage acquisition
  connect(ui->acquireCageLinkLabel, SIGNAL(linkActivated(QString)), this, SLOT(acquireCageLinkTriggered(QString)));
  connect(ui->sheepRandomizerModeComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(changeSheepRandomizerMode(int)));
  connect(ui->randomizeSheepPlanButton, SIGNAL(clicked()), this, SLOT(randomizeSheepCoordinates()));
  connect(ui->cageAcquisitionButton, SIGNAL(clicked()), this, SLOT(acquireCage()));
  connect(ui->copyAcquiredCageToInputButton, SIGNAL(clicked()), this, SLOT(copyAcquiredCageToInput()));
  connect(ui->saveCageAcquisitionPaths, SIGNAL(clicked()), this, SLOT(saveCageAcquisitionPaths()));
}

void MainWindow::runJob()
{
  if (ui->normalModeButton->isChecked())
  {
    start();
  }
  else
  {
    runStatistics();
  }
}

void MainWindow::runStatistics()
{
  QFile file("../statistics_result.txt");
  QTextStream stream(&file);

  statWidget->reset();
  ui->statTable->clearContents();
  ui->statTable->setColumnCount(4);
  ui->statTable->setRowCount(0);
  ui->statTable->setHorizontalHeaderLabels(QStringList({"Start configuration", "Goal region", "run #", "time"}));
  ui->statTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

  int nTries = statWidget->repeatsSpinBox->value();

  while (statWidget->getNextStart().size() > 0)
  {
    ui->nRobotsSpinBox->setValue(statWidget->getNextNRobots());

    for (int tries = 0; tries < nTries; ++tries)
    {
      file.open(QIODevice::Append);
      init();
      ui->logEdit->insertPlainText("Start #" + QString::number(statWidget->getNextStartID() + 1) + " + goal #" +
                                   QString::number(statWidget->getNextGoalID() + 1) + "; run #" +
                                   QString::number(tries + 1) + ": ");
      stream << "Start #" + QString::number(statWidget->getNextStartID() + 1) + " + goal #" +
                QString::number(statWidget->getNextGoalID() + 1) + "; run #" +
                QString::number(tries + 1) + ": ";
      double time = start();
      stream << time << "\n";

      ui->statTable->insertRow(ui->statTable->rowCount());
      std::stringstream ss;
      for (auto c : statWidget->getNextStart())
      {
        ss << c << " ";
      }
      auto item1 = new QTableWidgetItem(QString::fromStdString(ss.str()));
      ss.str("");
      for (auto c : statWidget->getNextGoal())
      {
        ss << c << " ";
      }
      auto item2 = new QTableWidgetItem(QString::fromStdString(ss.str()));
      auto item3 = new QTableWidgetItem(QString::number(tries));
      auto item4 = new QTableWidgetItem(QString::number(time, 'f', 2));
      ui->statTable->setItem(ui->statTable->rowCount() - 1, 0, item1);
      ui->statTable->setItem(ui->statTable->rowCount() - 1, 1, item2);
      ui->statTable->setItem(ui->statTable->rowCount() - 1, 2, item3);
      ui->statTable->setItem(ui->statTable->rowCount() - 1, 3, item4);
      file.flush();
      file.close();
    }
    statWidget->goToNextEntry();
  }
}

void MainWindow::changeEnvironment()
{
  auto filename = QFileDialog::getOpenFileName(this, "Select environment file supported by OMPL", "../QtHerding/meshes/environments/", "Collada files (*.dae)");
  if (filename != "")
  {
    loadEnvironment(filename);
  }
}

void MainWindow::loadEnvironment(QString filename)
{
  envMesh = filename.toStdString();
  statusBar()->showMessage(QString("Current environment: %1").arg(QFileInfo(filename).fileName()));
  changeScale();
}

void MainWindow::changeScale()
{
  initCanvas();
  drawCanvas();
  drawObstacles();
}

void MainWindow::checkStart()
{
  /*
  init();
  std::queue<double> inp;
  for (unsigned int i = 0; i < n; ++i)
  {
    inp.push(ui->startXEdit->document()->findBlockByLineNumber(i).text().toDouble());
    inp.push(ui->startYEdit->document()->findBlockByLineNumber(i).text().toDouble());
  }
  double eps;
  double maxEdge;
  int cycle;
  runVerification(inp, eps, maxEdge, cycle);
  std::cout << "Epsilon: " << eps << std::endl;
  */
}

void MainWindow::clearItems()
{
  if (goalRectItem)
  {
    delete goalRectItem;
    goalRectItem = nullptr;
  }
  if (fieldItem)
  {
    delete fieldItem;
    fieldItem = nullptr;
  }
  for (auto item : startItems)
  {
    delete item;
  }
  for (auto item : goalItems)
  {
    delete item;
  }
  for (auto item : configItems)
  {
    delete item;
  }
  for (auto item : pathItems)
  {
    delete item;
  }
  startItems.clear();
  goalItems.clear();
  configItems.clear();
  pathItems.clear();
}

void MainWindow::initCanvas()
{
  auto scene = new QGraphicsScene();
  ui->canvasView->setScene(scene);
  ui->canvasView->setBackgroundBrush(QBrush(Qt::lightGray));
  envItem = nullptr;
  goalRectItem = nullptr;
  fieldItem = nullptr;

  if (planning)
  {
    for (auto conf : configs)
    {
      for (int i = 0; i < n; ++i)
      {
        planning->getSpaceInformation()->getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(i)->as<base::SE2StateSpace>()->freeState(conf[i]);
      }
    }
    for (auto conf : path)
    {
      for (int i = 0; i < n; ++i)
      {
        planning->getSpaceInformation()->getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(i)->as<base::SE2StateSpace>()->freeState(conf[i]);
      }
    }
    configs.clear();
    path.clear();
    ui->confListWidget->clear();
    ui->pathListWidget->clear();
  }

  planning.reset(new HerdingPlanning(1));
  planning->setRobotMesh(robotMesh);
  planning->setEnvironmentMesh(envMesh);
  planning->getSpaceInformation()->getStateSpace()->setStateSamplerAllocator(allocHerdingSampler);
  base::ScopedState<base::CompoundStateSpace> goal(planning->getSpaceInformation());
  planning->setGoalState(goal);
  planning->setup();
}

void MainWindow::drawCanvas()
{
  double scale = ui->scaleSpinBox->value();
  auto bounds = planning->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(0)->as<ob::SE2StateSpace>()->getBounds();
  ui->canvasView->setSceneRect(bounds.low[0]*scale, bounds.low[1]*scale, (bounds.high[0]-bounds.low[0])*scale, (bounds.high[1]-bounds.low[1])*scale);
  envItem = ui->canvasView->scene()->addRect(bounds.low[0]*scale, bounds.low[1]*scale, (bounds.high[0]-bounds.low[0])*scale, (bounds.high[1]-bounds.low[1])*scale, QPen(Qt::black), QBrush(Qt::white));
  envItem->setZValue(Layer_Environment);
}

void MainWindow::drawObstacles()
{
  double scale = ui->scaleSpinBox->value();
  auto bounds = planning->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(0)->as<ob::SE2StateSpace>()->getBounds();
  auto checker = planning->getStateValidityChecker();
  int resolution = 100;
  double deltaX = (bounds.high[0] - bounds.low[0]) / resolution;
  double deltaY = (bounds.high[1] - bounds.low[1]) / resolution;
  auto state =static_cast<base::CompoundStateSpace::StateType*>(planning->getSpaceInformation()->allocState());
  obstacleItems.clear();
  for (int i = 1; i < resolution; ++i)
    for (int j = 1; j < resolution; ++j)
    {
      state->as<base::SE2StateSpace::StateType>(0)->setX(bounds.low[0] + i * deltaX);
      state->as<base::SE2StateSpace::StateType>(0)->setY(bounds.low[1] + j * deltaY);
      if (!checker->isValid(state))
      {
        auto obstacleItem = ui->canvasView->scene()->addEllipse(scale * (bounds.low[0] + i * deltaX) - scale * deltaX / 2,
            scale * (bounds.low[1] + j * deltaY) - scale * deltaY / 2,
            scale * deltaX, scale * deltaY, QPen(Qt::darkGray), QBrush(Qt::darkGray));
        obstacleItem->setZValue(Layer_Obstacles);
        if (!ui->showObstaclesCheckBox->isChecked())
          obstacleItem->hide();
        obstacleItems.push_back(obstacleItem);
      }
    }
}

void MainWindow::drawStart()
{
  double scale = ui->scaleSpinBox->value();
  startItems.clear();
  for (unsigned int i = 0; i < n; ++i)
  {
    startItems.push_back(ui->canvasView->scene()->addEllipse(starts[i]->getX()*scale, starts[i]->getY()*scale, 3, 3, QPen(Qt::black), QBrush(Qt::black)));
    if (!ui->showStartCheckBox->isChecked())
      startItems[i]->hide();
    startItems[i]->setZValue(Layer_Start);
  }
}

void MainWindow::drawGoal()
{
  double scale = ui->scaleSpinBox->value();
  goalItems.clear();
  auto coords = curWidget->getNextGoal();
  goalRectItem = ui->canvasView->scene()->addRect(fmin(coords[0], coords[2]) * scale,
                                                  fmin(coords[1], coords[3]) * scale,
                                                  fabs(coords[0] - coords[2]) * scale,
                                                  fabs(coords[1] - coords[3]) * scale,
                                                  QPen(Qt::blue), QBrush(Qt::blue));
  goalRectItem->setOpacity(0.2);
  goalRectItem->setZValue(Layer_Goal);
}

void MainWindow::drawConfig()
{
  if (ui->confListWidget->selectedItems().empty())
    return;

  int index = ui->confListWidget->selectedItems()[0]->data(Qt::UserRole).toInt();
  double scale = ui->scaleSpinBox->value();
  for (auto item : configItems)
  {
    delete item;
  }
  configItems.clear();
  for (unsigned int i = 0; i < n; ++i)
  {
    configItems.push_back(ui->canvasView->scene()->addEllipse(configs[index][i]->getX()*scale, configs[index][i]->getY()*scale, 3, 3, QPen(Qt::red), QBrush(Qt::red)));
    configItems[i]->setZValue(Layer_Configuration);
  }
  ui->showConfCheckBox->setChecked(true);
}

void MainWindow::drawPathConfig()
{
  if (ui->pathListWidget->selectedItems().empty())
    return;

  int index = ui->pathListWidget->selectedItems()[0]->data(Qt::UserRole).toInt();
  double scale = ui->scaleSpinBox->value();
  for (auto item : pathItems)
  {
    delete item;
  }
  pathItems.clear();
  for (unsigned int i = 0; i < n; ++i)
  {
    pathItems.push_back(ui->canvasView->scene()->addEllipse(path[index][i]->getX()*scale, path[index][i]->getY()*scale, 3, 3, QPen(Qt::darkGreen), QBrush(Qt::darkGreen)));
    pathItems[i]->setZValue(Layer_Path);
  }
  ui->showPathConfCheckBox->setChecked(true);
  drawPotentialField();
}

void MainWindow::drawPotentialField()
{
  if (ui->pathListWidget->selectedItems().empty())
    return;

  int index = ui->pathListWidget->selectedItems()[0]->data(Qt::UserRole).toInt();
  double scale = ui->scaleSpinBox->value();
  if (fieldItem != nullptr)
  {
    delete fieldItem;
  }

  QRectF r = envItem->boundingRect();
  QImage img(r.width() + 1, r.height() + 1, QImage::Format_RGB32);
  for (int x = r.left(); x < r.right(); ++x)
    for (int y = r.top(); y < r.bottom(); ++y)
    {
      double best_d = 1e10;
      for (auto p : pathItems)
      {
        QRectF path_rect = p->boundingRect();
        double d = (path_rect.left() - x)*(path_rect.left() - x) +
                   (path_rect.top() - y)*(path_rect.top() - y);
        if (d < best_d) best_d = d;
      }
      best_d = sqrt(best_d);
      double coef = exp(-best_d/50);
      QColor col(255, round((1 - coef) * 255), round((1 - coef) * 255));
      img.setPixel(x - r.left(), y - r.top(), col.rgb());
    }
  fieldItem = ui->canvasView->scene()->addPixmap(QPixmap::fromImage(img));
  fieldItem->setOpacity(double(ui->fieldOpacitySlider->value()) / 100.0);
  fieldItem->setZValue(Layer_PotentialField);
  fieldItem->setPos(r.left(), r.top());
  if (!ui->showFieldCheckBox->isChecked())
    fieldItem->hide();
}

void MainWindow::showStart(bool bShow)
{
  for (auto item : startItems)
  {
    if (bShow)
      item->show();
    else
      item->hide();
  }
}

void MainWindow::showGoal(bool bShow)
{
  for (auto item : goalItems)
  {
    if (bShow)
      item->show();
    else
      item->hide();
  }
  if (goalRectItem)
  {
    if (bShow)
      goalRectItem->show();
    else
      goalRectItem->hide();
  }
}

void MainWindow::showConfig(bool bShow)
{
  for (auto item : configItems)
  {
    if (bShow)
      item->show();
    else
      item->hide();
  }
}

void MainWindow::showPathConfig(bool bShow)
{
  for (auto item : pathItems)
  {
    if (bShow)
      item->show();
    else
      item->hide();
  }
}

void MainWindow::showObstacles(bool bShow)
{
  for (auto item : obstacleItems)
  {
    if (bShow)
      item->show();
    else
      item->hide();
  }
}

void MainWindow::showPotentialField(bool bShow)
{
  if (fieldItem != nullptr)
  {
    if (bShow)
      fieldItem->show();
    else
      fieldItem->hide();
    ui->fieldOpacitySlider->setEnabled(bShow);
  }
}

void MainWindow::setPotentialFieldOpacity(int percent)
{
  fieldItem->setOpacity(double(percent) / 100.0);
}

void MainWindow::setMode()
{
  bool bSingle = ui->normalModeButton->isChecked();
  singleWidget->hide();
  statWidget->hide();
  singleWidget->setVisible(bSingle);
  statWidget->setVisible(!bSingle);

  ui->nRobotsSpinBox->setEnabled(bSingle);
  ui->static3->setVisible(bSingle);
  ui->pathListWidget->setVisible(bSingle);

  if (bSingle)
  {
    curWidget = singleWidget;
  }
  else
  {
    curWidget = statWidget;
  }
}

void MainWindow::doubleClickedConf(QModelIndex index)
{
  int ind = ui->confListWidget->item(index.row())->data(Qt::UserRole).toInt();
  std::stringstream ss;
  for (unsigned int i = 0; i < n; ++i)
  {
    double x = configs[ind][i]->getX();
    double y = configs[ind][i]->getY();
    ss << x << " " << y << " ";
  }
  statWidget->addStart(QString::fromStdString(ss.str()));
}

void MainWindow::copyTableDataToClipboard()
{
  QByteArray byteArray;

  for (int row = 0; row < ui->statTable->rowCount(); ++row)
  {
    for (int col = 0; col < ui->statTable->columnCount(); ++col)
    {
      byteArray.append(ui->statTable->item(row, col)->text());
      byteArray.append("\t");
    }
    byteArray.append("\r\n");
  }
  QMimeData *pMimeData = new QMimeData();
  pMimeData->setData("text/plain", byteArray);
  QApplication::clipboard()->setMimeData(pMimeData);
}

void MainWindow::savePathToFile()
{
  auto filename = QFileDialog::getSaveFileName(this, "Save robot paths to text file", ".", "Text files (*.txt)");
  if (filename != "")
  {
    if (!filename.endsWith(".txt", Qt::CaseInsensitive))
    {
      filename += ".txt";
    }
    QFile file(filename);
    if (file.open(QIODevice::WriteOnly))
    {
      n = ui->nRobotsSpinBox->value();
      QTextStream stream(&file);
      stream << "# File format: 2nd line - number of robots (N); 3rd line - number of time points; 4th-... lines - sets of N coordinate pairs x_i, y_i" << endl;
      stream << n << endl;
      stream << ui->pathListWidget->count() << endl;

      for (int i = 0; i < ui->pathListWidget->count(); ++i)
      {
        int path_conf_index = ui->pathListWidget->item(i)->data(Qt::UserRole).toInt();
        for (unsigned int j = 0; j < n; ++j)
        {
          stream << path[path_conf_index][j]->getX() << " " << path[path_conf_index][j]->getY() << " ";
        }
        stream << endl;
      }

      file.flush();
      file.close();
    }
  }
}

//
// Cage acquisition part
//

void MainWindow::acquireCageLinkTriggered(const QString &link)
{
  ui->tabWidget->setCurrentIndex(3);
}

void MainWindow::changeSheepRandomizerMode(int index)
{
  switch (index)
  {
    case 0: {
      ui->sheepModeLabel->setText("<html><head/><body><p><span style=\" font-size:9pt;\">x</span><span style=\" font-size:9pt; vertical-align:sub;\">center</span><span style=\" font-size:9pt;\">, y</span><span style=\" font-size:9pt; vertical-align:sub;\">center</span><span style=\" font-size:9pt;\">, r:</span></p></body></html>");
      break;
    }
    case 1: {
      ui->sheepModeLabel->setText("<html><head/><body><p><span style=\" font-size:8pt;\">x</span><span style=\" font-size:8pt; vertical-align:sub;\">min</span><span style=\" font-size:8pt;\">, y</span><span style=\" font-size:8pt; vertical-align:sub;\">min</span><span style=\" font-size:8pt;\">, x</span><span style=\" font-size:8pt; vertical-align:sub;\">max</span><span style=\" font-size:8pt;\">, y</span><span style=\" font-size:8pt; vertical-align:sub;\">max</span><span style=\" font-size:8pt;\">:</span></p></body></html>");
      break;
    }
  }
}

void MainWindow::randomizeSheepCoordinates()
{
  auto randomizer_text = ui->sheepRandomizerEdit->text();
  vector<QString> splitters = {" ", ",", ", "};
  int expected_count = (ui->sheepRandomizerModeComboBox->currentIndex() == 0)
                        ? 3  // circle
                        : 4; // box
  int max_count = 0;
  QString good_splitter = "";
  for (auto splitter : splitters)
  {
    auto coords = randomizer_text.split(splitter);
    int count = 0;
    for (auto coord : coords)
    {
      if (coord != "")
      {
        count++;
      }
    }
    max_count = max(max_count, count);
    if (count == expected_count)
    {
      good_splitter = splitter;
      max_count = count;
      break;
    }
  }
  if (max_count != expected_count)
  {
    QMessageBox msgBox;
    msgBox.setText(QString("Too few numbers in randomizer input (given %1, expected %2)!").arg(max_count).arg(expected_count));
    msgBox.exec();
    return;
  }

  auto coords = randomizer_text.split(good_splitter);
  int sheep_count = ui->sheepCountSpinBox->value();
  ui->cagePlanSheepCoordEdit->clear();
  switch (ui->sheepRandomizerModeComboBox->currentIndex())
  {
    case 0:
    {
      // circle
      double x = coords[0].toDouble();
      double y = coords[1].toDouble();
      double r = coords[2].toDouble();
      for (int i = 0; i < sheep_count; ++i)
      {
        double rand_r = double(rand() % 1000) / 1000.0 * r;
        double rand_phi = double(rand() % 1000) / 1000.0 * 2 * M_PIl;
        ui->cagePlanSheepCoordEdit->insertPlainText(QString("%1 %2\n").arg(x + rand_r * sin(rand_phi)).arg(y + rand_r * cos(rand_phi)));
      }
      break;
    }
    case 1:
    {
      // box
      double x1 = coords[0].toDouble();
      double y1 = coords[1].toDouble();
      double x2 = coords[2].toDouble();
      double y2 = coords[3].toDouble();
      for (int i = 0; i < sheep_count; ++i)
      {
        double rand_x = double(rand() % 1000) / 1000.0 * fabs(x2 - x1);
        double rand_y = double(rand() % 1000) / 1000.0 * fabs(y2 - y1);
        ui->cagePlanSheepCoordEdit->insertPlainText(QString("%1 %2\n").arg(min(x1, x2) + rand_x).arg(min(y1, y2) + rand_y));
      }
      break;
    }
  }
}

double determinant(vector<double> m)
{
    return  m[0] * m[4] * m[8] - m[0] * m[5] * m[7] -
            m[1] * m[3] * m[8] + m[1] * m[5] * m[6] +
            m[2] * m[3] * m[7] - m[2] * m[4] * m[6];
}

void getMinimumCoveringCircle(vector<pair<double, double>> coords, double &x, double &y, double &r)
{
  if (coords.empty())
  {
    cout << "Warning: given empty coordinates for getMinimumCoveringCircle()!" << endl;
    x = 0.0;
    y = 0.0;
    r = 0.0;
    return;
  }

  double cur_x, cur_y, cur_r;
  double r2 = numeric_limits<double>::max();

  //
  // The slowest N^4 solution -- just for the sake of coding time saving
  //

  // try 2-point circles (built on diameter)
  for (int i = 0; i < coords.size() - 1; ++i)
  {
    for (int j = i + 1; j < coords.size(); ++j)
    {
      cur_x = (coords[i].first + coords[j].first) / 2;
      cur_y = (coords[i].second + coords[j].second) / 2;
      double cur_r2 = (coords[i].first - cur_x) * (coords[i].first - cur_x) +
                      (coords[i].second - cur_y) * (coords[i].second - cur_y);
      if (cur_r2 >= r2)
      {
        continue;
      }

      bool covers_all_points = true;
      for (int k = 0; k < coords.size(); ++k)
      {
        if ((cur_x - coords[k].first) * (cur_x - coords[k].first) +
            (cur_y - coords[k].second) * (cur_y - coords[k].second) > cur_r2)
        {
          covers_all_points = false;
          break;
        }
      }
      if (covers_all_points)
      {
        r2 = cur_r2;
        x = cur_x;
        y = cur_y;
        r = sqrt(r2);
      }
    }
  }

  // try 3-point circles
  for (int i = 0; i < coords.size() - 2; ++i)
  {
    double x1 = coords[i].first;
    double y1 = coords[i].second;
    for (int j = i + 1; j < coords.size() - 1; ++j)
    {
      double x2 = coords[j].first;
      double y2 = coords[j].second;
      for (int k = j + 1; k < coords.size(); ++k)
      {
        double x3 = coords[k].first;
        double y3 = coords[k].second;

        // calculate minors of:
        // [x *x  + y *y,  x,  y,  1]
        // [x1*x1 + y1*y1, x1, y1, 1]
        // [x2*x2 + y2*y2, x2, y2, 1]
        // [x3*x3 + y3*y3, x3, y3, 1]
        auto m11 = determinant({x1, y1, 1, x2, y2, 1, x3, y3, 1});
        if (m11 == 0.0)
        {
          // 3 points lie on a line
          continue;
        }
        auto m12 = determinant({x1*x1 + y1*y1, y1, 1, x2*x2 + y2*y2, y2, 1, x3*x3 + y3*y3, y3, 1});
        auto m13 = determinant({x1*x1 + y1*y1, x1, 1, x2*x2 + y2*y2, x2, 1, x3*x3 + y3*y3, x3, 1});
        cur_x = 0.5 * m12 / m11;
        cur_y = -0.5 * m13 / m11;
        double cur_r2 = (x1 - cur_x) * (x1 - cur_x) +
                        (y1 - cur_y) * (y1 - cur_y);
        if (cur_r2 >= r2)
        {
          continue;
        }

        bool covers_all_points = true;
        for (int l = 0; l < coords.size(); ++l)
        {
          if ((cur_x - coords[l].first) * (cur_x - coords[l].first) +
              (cur_y - coords[l].second) * (cur_y - coords[l].second) > cur_r2 + 0.000001)
          {
            covers_all_points = false;
            break;
          }
        }
        if (covers_all_points)
        {
          r2 = cur_r2;
          x = cur_x;
          y = cur_y;
          r = sqrt(r2);
        }
      }
    }
  }
  assert(r2 != numeric_limits<double>::max());
}

vector<Coordinates> CreateCageAcquisitionPaths(double x, double y, double r_caging_circle,
                                               int polygon_rotation, int n_optimal, Coordinates robots,
                                               vector<double> polar_angles,
                                               vector<int> robot_target_vertices) {
  auto N = robots.size();
  vector<Coordinates> cageAcquisitionPaths = {};
  // initial configuration
  cageAcquisitionPaths.push_back({});
  for (int i = 0; i < N; ++i)
  {
    cageAcquisitionPaths[0].push_back(robots[i]);
  }
  // first step -- translation to caging circle
  cageAcquisitionPaths.push_back({});
  for (int i = 0; i < N; ++i)
  {
    if (robot_target_vertices[i] != -1) {
      cageAcquisitionPaths[1].push_back(make_pair(x + r_caging_circle * sin(polar_angles[i]),
                                                  y + r_caging_circle * cos(polar_angles[i])));
    } else {
      cageAcquisitionPaths[1].push_back(make_pair(std::numeric_limits<double>::max(),
                                                  std::numeric_limits<double>::max()));
    }
  }
  // Note: discretization of robot paths takes place here
  // further steps -- moving to the desired point on caging circle
  for (int j = 0; j < kCageAcquisitionMilestonesCount; ++j)
  {
    cageAcquisitionPaths.push_back({});
    for (int i = 0; i < N; ++i)
    {
      if (robot_target_vertices[i] != -1) {
        double vertex_angle = 2 * M_PIl / n_optimal * robot_target_vertices[i] +
                              polygon_rotation * kSearchResolution;
        double milestone_angle = polar_angles[i] + (j + 1) * (vertex_angle - polar_angles[i]) / (kCageAcquisitionMilestonesCount);
        cageAcquisitionPaths[j + 2].push_back(make_pair(x + r_caging_circle * sin(milestone_angle),
                                                        y + r_caging_circle * cos(milestone_angle)));
      } else {
        cageAcquisitionPaths[j + 2].push_back(make_pair(std::numeric_limits<double>::max(),
                                                        std::numeric_limits<double>::max()));
      }
    }
  }

  return cageAcquisitionPaths;
}

// v - robot speed, u - sheep speed
// x, y, r - parameters of sheep circle

vector<Coordinates> solveCageAcquisitionMinTime(double safety_margin, double v, double u,
                                                double x, double y, double r,
                                                double min_r_cage, double max_r_cage,
                                                Coordinates robots,
                                                vector<double> polar_angles,
                                                function<void(Coordinates)> output)
{
  Coordinates robots_final;

  // binary search of cage size (cage size = radius of cage inner circle = r_cage)
  double solution_time = -1;
  int solution_polygon_rotation;
  double solution_caging_circle;
  int N = robots.size();
  vector<int> solution_robot_vertex_positions(N);
  while (true)
  {
    double r_cage = (min_r_cage + max_r_cage) / 2;
    double r_caging_circle = r_cage / (1 / sin(M_PIl / N) - 1) + r_cage + safety_margin;

    // time that will take sheep to expand to a circle of radius r_cage
    double t = (r_cage - r) / u;

    // try to perform 2 steps of cage formation in time <= t:
    // 1) reaching "caging circle" - circumscribed circle of caging polygon
    // 2) moving along "caging circle" to accomodate in a regular polygon
    vector<double> times_1_step(N);
    for (int i = 0; i < N; ++i)
    {
      double rx = robots[i].first;
      double ry = robots[i].second;
      times_1_step[i] = fabs(sqrt((rx - x) * (rx - x) + (ry - y) * (ry - y)) - r_caging_circle);
    }

    // try all orientations of regular polygon as target cage (not exact, discrete search)
    double best_max_time = numeric_limits<double>::max();
    int best_polygon_rotation = 0;
    vector<int> best_robot_vertex_positions(N);
    for (int i = 0; i < 2 * M_PIl / N / kSearchResolution; ++i)
    {
      double polygon_rotation = i * kSearchResolution;
      double max_time = 0;
      vector<int> robot_moved_to_vertex(N, -1);
      for (int j = 0; j < N; ++j)
      {
        double vertex_angle = 2 * M_PIl / N * j + polygon_rotation;

        // for each polygon vertex find closest robot on a caging circle
        int closest_robot = -1;
        for (int k = 0; k < N; ++k)
        {
          if (robot_moved_to_vertex[k] == -1)
          {
            if (closest_robot == -1)
            {
              closest_robot = k;
            }
            else
            {
              if (fabs(polar_angles[closest_robot] - vertex_angle) >
                  fabs(polar_angles[k] - vertex_angle))
              {
                  closest_robot = k;
              }
            }
          }
        }
        assert(closest_robot != -1);

        // time taken by selected robot for two steps, including reaching current polygon vertex
        robot_moved_to_vertex[closest_robot] = j;
        double time_2_step = fabs(polar_angles[closest_robot] - vertex_angle) * r_caging_circle / v;
        double overall_time = times_1_step[closest_robot] + time_2_step;
        if (overall_time > max_time)
        {
          max_time = overall_time;
        }
      }
      if (max_time < best_max_time)
      {
        best_max_time = max_time;
        best_polygon_rotation = i;
        for (int j = 0; j < N; ++j)
        {
          best_robot_vertex_positions[j] = robot_moved_to_vertex[j];
        }
      }
    }
    if (best_max_time < t)
    {
      max_r_cage = r_cage;
      solution_time = best_max_time;
      solution_polygon_rotation = best_polygon_rotation;
      solution_caging_circle = r_caging_circle;
      for (int j = 0; j < N; ++j)
      {
        solution_robot_vertex_positions[j] = best_robot_vertex_positions[j];
      }
    }
    else
    {
      min_r_cage = r_cage;
    }
    if (max_r_cage - min_r_cage <= 0.0001)
    {
      if (solution_time != -1)
      {
        for (int i = 0; i < N; ++i)
        {
          double vertex_angle = 2 * M_PIl / N * solution_robot_vertex_positions[i] +
                                solution_polygon_rotation * kSearchResolution;
          double target_x = x + solution_caging_circle * sin(vertex_angle);
          double target_y = y + solution_caging_circle * cos(vertex_angle);
          robots_final.push_back(make_pair(target_x, target_y));
        }
        output(robots_final);
      }
      else
      {
        QMessageBox msgBox;
        msgBox.setText(QString("Cannot form a cage! Try to decrease safety margin, alternatively use more " \
                               "robots or put them in a different starting position."));
        msgBox.exec();
      }
      break;
    }
  }

  if (solution_time != -1) {
    return CreateCageAcquisitionPaths(x, y, solution_caging_circle, solution_polygon_rotation,
                                      N, robots, polar_angles, solution_robot_vertex_positions);
  } else {
    return {};
  }
}

vector<Coordinates> solveCageAcquisitionMinRobots(double safety_margin, double v, double u,
                                                  double x, double y, double r,
                                                  double min_r_cage, double max_r_cage,
                                                  Coordinates robots,
                                                  vector<double> polar_angles,
                                                  function<void(Coordinates)> output)
{
  Coordinates robots_final;
  struct Solution {
    double r_caging_circle;
    int polygon_rotation;
    vector<int> robot_target_vertices;
  };

  int N = robots.size();
  int best_robot_number = N + 1;
  map<int, Solution> solutions;

  // binary search of cage size (cage size = radius of cage inner circle = r_cage)
  while (true)
  {
    double r_cage = (min_r_cage + max_r_cage) / 2;
    int current_best_robot_number = N + 1;
    for (int m = 3; m <= N; ++m) {
      double r_caging_circle = r_cage / (1 / sin(M_PIl / m) - 1) + r_cage + safety_margin;

      // time that will take sheep to expand to a circle of radius r_cage
      double t = (r_cage - r) / u;

      // try to perform 2 steps of cage formation in time <= t:
      // 1) reaching "caging circle" - circumscribed circle of caging polygon
      // 2) moving along "caging circle" to accomodate in a regular polygon
      vector<double> times_1_step(N);
      for (int i = 0; i < N; ++i)
      {
        double rx = robots[i].first;
        double ry = robots[i].second;
        times_1_step[i] = fabs(sqrt((rx - x) * (rx - x) + (ry - y) * (ry - y)) - r_caging_circle);
      }

      // try all orientations of regular polygon as target cage (not exact, discrete search)
      double best_max_time = numeric_limits<double>::max();
      int best_polygon_rotation = -1;
      vector<int> best_robot_vertex_positions(N);
      for (int i = 0; i < 2 * M_PIl / m / kSearchResolution; ++i)
      {
        double polygon_rotation = i * kSearchResolution;
        double max_time = 0;

        vector<vector<pair<int, double>>> robot_times(m);
        vector<int> sorted_vertices(m);
        for (int j = 0; j < m; ++j)
        {
          sorted_vertices[j] = j;
          double vertex_angle = 2 * M_PIl / m * j + polygon_rotation;

          for (int k = 0; k < N; ++k)
          {
            double time_k = fabs(polar_angles[k] - vertex_angle) * r_caging_circle / v + times_1_step[k];
            robot_times[j].push_back(make_pair(k, time_k));
          }

          // Sort robots in an increasing order of times to reach this vertex
          sort(robot_times[j].begin(), robot_times[j].end(), [](pair<int, double> A, pair<int, double> B) {
            return A.second < B.second;
          });
        }

        // Sort polygon vertices in a decreasing order of the fastest robot time
        sort(sorted_vertices.begin(), sorted_vertices.end(), [&](int A, int B) {
          if (robot_times[A][0].second == robot_times[B][0].second) {
            return robot_times[A][1].second < robot_times[B][1].second;
          } else {
            return robot_times[A][0].second > robot_times[B][0].second;
          }
        });
        vector<int> robot_moved_to_vertex(N, -1);

        for (int l = 0; l < m; ++l) {
          int j = sorted_vertices[l];
          for (int k = 0; k < N; ++k) {
            if (robot_moved_to_vertex[robot_times[j][k].first] == -1) {
              robot_moved_to_vertex[robot_times[j][k].first] = j;
              if (robot_times[j][k].second > max_time) {
                max_time = robot_times[j][k].second;
              }
              break;
            }
          }
        }

        if (max_time < t)
        {
          if (max_time < best_max_time) {
            best_max_time = max_time;
            best_polygon_rotation = i;
            for (int k = 0; k < N; ++k)
            {
              best_robot_vertex_positions[k] = robot_moved_to_vertex[k];
            }
          }
        }
      }

      if (best_max_time < t) { // or simply best_max_time != inf
        // only update it once! - with minimal found m
        if (current_best_robot_number == N + 1) {
          current_best_robot_number = m;
        }
        bool new_solution = false;
        if (solutions.find(m) == solutions.end()) {
          new_solution = true;
        } else if (solutions[m].r_caging_circle > r_caging_circle) {
          new_solution = true;
        }
        if (new_solution) {
          solutions[m] = Solution();
          solutions[m].r_caging_circle = r_caging_circle;
          solutions[m].polygon_rotation = best_polygon_rotation;
          solutions[m].robot_target_vertices.resize(N);
          for (int k = 0; k < N; ++k)
          {
            solutions[m].robot_target_vertices[k] = best_robot_vertex_positions[k];
          }
        }
      }
    }

    if (current_best_robot_number >= best_robot_number) {
      min_r_cage = r_cage;
    } else {
      best_robot_number = current_best_robot_number;
      max_r_cage = r_cage;
    }
    if (max_r_cage - min_r_cage <= 0.0001)
    {
      if (!solutions.empty())
      {
        for (int i = 0; i < N; ++i)
        {
          if (solutions[best_robot_number].robot_target_vertices[i] != -1) {
            // Robot is included
            double vertex_angle = 2 * M_PIl / best_robot_number * solutions[best_robot_number].robot_target_vertices[i] +
                                  solutions[best_robot_number].polygon_rotation * kSearchResolution;
            double target_x = x + solutions[best_robot_number].r_caging_circle * sin(vertex_angle);
            double target_y = y + solutions[best_robot_number].r_caging_circle * cos(vertex_angle);
            robots_final.push_back(make_pair(target_x, target_y));
          } else {
            robots_final.push_back(make_pair(std::numeric_limits<double>::max(),
                                             std::numeric_limits<double>::max()));
          }
        }
        output(robots_final);
      }
      else
      {
        QMessageBox msgBox;
        msgBox.setText(QString("Cannot form a cage! Try to decrease safety margin, alternatively use more" \
                               "robots or put them in a different starting position."));
        msgBox.exec();
      }
      break;
    }
  }

  if (!solutions.empty()) {
    return CreateCageAcquisitionPaths(x, y, solutions[best_robot_number].r_caging_circle,
                                      solutions[best_robot_number].polygon_rotation, best_robot_number,
                                      robots, polar_angles, solutions[best_robot_number].robot_target_vertices);
  } else {
    return {};
  }
}

void MainWindow::acquireCage()
{
  Coordinates sheep;
  Coordinates robots;

  // parse sheep coordinates
  for (int i = 0; i < ui->cagePlanSheepCoordEdit->document()->lineCount(); ++i)
  {
    auto line = ui->cagePlanSheepCoordEdit->document()->findBlockByLineNumber(i).text();
    if (line == "")
    {
      break;
    }
    auto tokens = line.split(' ');
    if (tokens.size() < 2)
    {
      QMessageBox msgBox;
      msgBox.setText(QString("Too few numbers in line %1").arg(i + 1));
      msgBox.exec();
      return;
    }
    sheep.push_back(make_pair(tokens[0].toDouble(), tokens[1].toDouble()));
  }

  // parse robot coordinates
  if (ui->nRobotsSpinBox->value() != ui->cagePlanRobotCoordEdit->document()->lineCount())
  {
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Different number of robots",
      QString("Number of lines in robot input (%1) differs from the number of robots" \
              "set explicitly (%2). Do you want to update number of robots with %1\?").arg(ui->cagePlanRobotCoordEdit->document()->lineCount()).arg(ui->nRobotsSpinBox->value()),
              QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
      ui->nRobotsSpinBox->setValue(ui->cagePlanRobotCoordEdit->document()->lineCount());
    }
    else
    {
      return;
    }
  }
  for (int i = 0; i < ui->cagePlanRobotCoordEdit->document()->lineCount(); ++i)
  {
    auto line = ui->cagePlanRobotCoordEdit->document()->findBlockByLineNumber(i).text();
    auto tokens = line.split(' ');
    if (tokens.size() < 2)
    {
      QMessageBox msgBox;
      msgBox.setText(QString("Too few numbers in line %1").arg(i + 1));
      msgBox.exec();
      return;
    }
    robots.push_back(make_pair(tokens[0].toDouble(), tokens[1].toDouble()));
  }
  int N = robots.size();
  if (N < 3)
  {
    QMessageBox msgBox;
    msgBox.setText(QString("Too few robots (%1), need at least 3!").arg(N));
    msgBox.exec();
    return;
  }

  double x, y, r;
  getMinimumCoveringCircle(sheep, x, y, r);

  double min_r_cage = r;
  double max_r_cage = r;
  // approximate max_r_cage as twice distance to the most distant robot
  for (auto robot : robots)
  {
    max_r_cage = max(max_r_cage,
                     sqrt((robot.first - x) * (robot.first - x) +
                          (robot.second - y) * (robot.second - y)) * 2);
  }

  vector<double> polar_angles(N);
  vector<int> robots_sorted_by_angle(N);
  for (int i = 0; i < N; ++i)
  {
    double rx = robots[i].first;
    double ry = robots[i].second;
    polar_angles[i] = asin((rx - x) / sqrt((rx - x) * (rx - x) + (ry - y) * (ry - y)));

    if (ry < y)
    {
      polar_angles[i] = M_PIl - polar_angles[i];
    }
    if (polar_angles[i] < 0)
    {
      polar_angles[i] = 2 * M_PIl + polar_angles[i];
    }
    robots_sorted_by_angle[i] = i;
  }
  /*
  sort(robots_sorted_by_angle.begin(), robots_sorted_by_angle.end(), [&](int a, int b)
  {
    return polar_angles[a] < polar_angles[b];
  });
  */

  // set robot speed to 1, does not change anything but time measure
  double safety_margin = ui->cageSafetyMarginSpinBox->value();
  double v = 1.0;
  double u = ui->speedRatioSpinBox->value() * v;

  auto output = [&](Coordinates result) {
    outputAcquiredCage(result);
    displayAcquiredCage(sheep, robots, result);
  };

  cageAcquisitionPaths = (ui->optimizeRobotNumberCheck->isChecked())
    ? solveCageAcquisitionMinRobots(safety_margin, v, u, x, y, r, min_r_cage, max_r_cage, robots,
                                    polar_angles, output)
    : solveCageAcquisitionMinTime(safety_margin, v, u, x, y, r, min_r_cage, max_r_cage, robots,
                                  polar_angles, output);
}

void MainWindow::copyAcquiredCageToInput()
{
  QStringList cage_coords = {};
  for (int i = 0; i < ui->cageResultRobotCoordEdit->document()->lineCount(); ++i)
  {
    cage_coords.append(ui->cageResultRobotCoordEdit->document()->findBlockByLineNumber(i).text());
  }
  curWidget->setStarts(cage_coords);
}

bool IsCoordinateValid(pair<double, double> coordinate) {
  return ((coordinate.first != std::numeric_limits<double>::max()) &&
          (coordinate.second != std::numeric_limits<double>::max()));
}

void MainWindow::outputAcquiredCage(Coordinates &robots_final)
{
  ui->cageResultRobotCoordEdit->clear();
  for (auto robot : robots_final)
  {
    if (IsCoordinateValid(robot)) {
      ui->cageResultRobotCoordEdit->insertPlainText(QString("%1 %2\n").arg(robot.first).arg(robot.second));
    } else {
      ui->cageResultRobotCoordEdit->insertPlainText(QString("n/a n/a\n"));
    }
  }
}

void MainWindow::displayAcquiredCage(Coordinates &sheep, Coordinates &robots_start, Coordinates &robots_final)
{
  auto scene = new QGraphicsScene();
  ui->canvasPreview->setScene(scene);
  ui->canvasPreview->setBackgroundBrush(QBrush(Qt::white));
  for (auto item : cagePlanSheepItems)
  {
    if (item) delete item;
  }
  for (auto item : cagePlanRobotStartItems)
  {
    if (item) delete item;
  }
  for (auto item : cagePlanRobotFinalItems)
  {
    if (item) delete item;
  }
  for (auto item : cagePlanPathsItems)
  {
    if (item) delete item;
  }
  cagePlanSheepItems.clear();
  cagePlanRobotStartItems.clear();
  cagePlanRobotFinalItems.clear();
  cagePlanPathsItems.clear();

  double min_x = 100000, max_x = -100000, min_y = 100000, max_y = -100000;
  for (auto single_sheep : sheep)
  {
    min_x = min(min_x, single_sheep.first);
    min_y = min(min_y, single_sheep.second);
    max_x = max(max_x, single_sheep.first);
    max_y = max(max_y, single_sheep.second);
  }
  for (auto robot : robots_start)
  {
    min_x = min(min_x, robot.first);
    min_y = min(min_y, robot.second);
    max_x = max(max_x, robot.first);
    max_y = max(max_y, robot.second);
  }
  for (auto robot : robots_final)
  {
    if (IsCoordinateValid(robot)) {
      min_x = min(min_x, robot.first);
      min_y = min(min_y, robot.second);
      max_x = max(max_x, robot.first);
      max_y = max(max_y, robot.second);
    }
  }
  QRectF exactRect(min_x, min_y, max_x - min_x, max_y - min_y);
  double margin = std::min(exactRect.width(), exactRect.height()) / 10;
  exactRect.adjust(-margin, -margin, margin, margin);
  ui->canvasPreview->setSceneRect(exactRect);
  ui->canvasPreview->fitInView(exactRect, Qt::KeepAspectRatio);
  ui->canvasPreview->centerOn(exactRect.center());

  double point_radius = std::min(max_x - min_x, max_y - min_y) / 100;
  for (auto single_sheep : sheep)
  {
    cagePlanSheepItems.push_back(
          ui->canvasPreview->scene()->addEllipse(single_sheep.first - point_radius,
                single_sheep.second - point_radius,
                point_radius * 2, point_radius * 2, QPen(Qt::blue), QBrush(Qt::blue)));
  }

  QPen line_pen;
  line_pen.setColor(Qt::lightGray);
  line_pen.setStyle(Qt::DashLine );
  line_pen.setWidth(std::max(1, static_cast<int>(point_radius)));
  for (size_t i = 0; i < robots_final.size(); ++i)
  {
    auto robot = robots_final[i];
    if (IsCoordinateValid(robot)) {
      cagePlanPathsItems.push_back(
            ui->canvasPreview->scene()->addLine(robots_start[i].first, robots_start[i].second,
                  robot.first, robot.second, line_pen));
      cagePlanRobotFinalItems.push_back(
            ui->canvasPreview->scene()->addEllipse(robot.first - point_radius,
                  robot.second - point_radius,
                  point_radius * 2, point_radius * 2, QPen(Qt::red), QBrush(Qt::red)));
    }
  }
  // either keep this last to not be overdrawn by lines, or use Z values
  for (size_t i = 0; i < robots_start.size(); ++i)
  {
    auto robot = robots_start[i];
    auto color = Qt::yellow;
    if (!IsCoordinateValid(robots_final[i])) {
      color = Qt::darkGray;
    }
    cagePlanRobotStartItems.push_back(
          ui->canvasPreview->scene()->addEllipse(robot.first - point_radius,
                robot.second - point_radius,
                point_radius * 2, point_radius * 2, QPen(color), QBrush(color)));
  }
}

void MainWindow::saveCageAcquisitionPaths()
{
  auto filename = QFileDialog::getSaveFileName(this, "Save cage acquisition paths to text file", ".", "Text files (*.txt)");
  if (filename != "")
  {
    if (!filename.endsWith(".txt", Qt::CaseInsensitive))
    {
      filename += ".txt";
    }
    QFile file(filename);
    if (file.open(QIODevice::WriteOnly))
    {
      n = ui->nRobotsSpinBox->value();

      QTextStream stream(&file);
      stream << "# File format: 2nd line - number of robots (N); 3rd line - number of entries; 4th-... lines - sets of N coordinate pairs x_i, y_i" << endl;
      stream << n << endl;

      stream << cageAcquisitionPaths.size() << endl;

      for (int i = 0; i < cageAcquisitionPaths.size(); ++i)
      {
        for (int j = 0; j < n; ++j)
        {
          stream << cageAcquisitionPaths[i][j].first << " " << cageAcquisitionPaths[i][j].second << " ";
        }
        stream << endl;
      }

      file.flush();
      file.close();
    }
  }
}

MainWindow::~MainWindow()
{
  delete ui;
}

/*
-2 -1
0 2
2 2
2 1

-2 -1
0 2
2 2
2 1
-2 -2
2 -2
-3 -1
0 3

-0.0515193 -0.109484
0.127663 0.0293795
-0.178185 -0.560352
0.138508 -0.0673158
-0.309945 -0.860906
-0.0003453 -0.00498806

-0.461998 0.544384
-0.083483 -0.103535
-0.785573 -0.255248
-0.705396 0.637822
0.195216 0.383133
-0.856653 -0.0864114
 * */


/*
 * 0.5828 -4.61333
-1.9061 -2.4257
4.89516 -0.49378
-1.64635 1.19614
-2.2485 1.05806
0.94884 1.8622

-3 -3
-5 -4
5 5
3 3
0 4
7 0
10 10
*/
