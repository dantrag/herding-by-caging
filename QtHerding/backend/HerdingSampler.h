#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <iostream>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class HerdingSampler : public ob::CompoundStateSampler
{
public:
  HerdingSampler(const ob::StateSpace *space) : CompoundStateSampler(space)
  {
    equilibriumD = 0;
  }

  virtual void sampleUniform(ob::State *state)
  {
  }

  virtual void sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
  {
    auto statetype = static_cast<ob::CompoundStateSpace::StateType*>(state);
    auto currenttype = static_cast<const ob::CompoundStateSpace::StateType*>(near);
    auto bounds = space_->as<ob::CompoundStateSpace>()->getSubspace(0)->as<ob::SE2StateSpace>()->getBounds();

    int n = space_->as<ob::CompoundStateSpace>()->getSubspaceCount();
    double expansion = 1.0;
    int count = 0;
    std::vector<std::pair<double, double> > deltas(n);
    std::vector<std::pair<double, double> > newCoords(n);
    assert(equilibriumD >= 1e-10);
    while (true)
    {
      count++;
      if (count % 10 ==  0)
      {
        expansion *= 1.1;
      }
      if (count % 100 == 0)
      {
        std::cout << "Cannot find valid near sample after 100 tries!" << std::endl;
        for (int i = 0; i < n; ++i)
        {
          double X = currenttype->as<ob::SE2StateSpace::StateType>(i)->getX();
          double Y = currenttype->as<ob::SE2StateSpace::StateType>(i)->getY();
          newCoords[i] = std::make_pair(X, Y);
        }
        break;
      }
      std::vector<double> val(2);
      for (int i = 0; i < n; ++i)
      {
        rng_.uniformInBall(distance*fractionNear*expansion, val);
        deltas[i] = std::make_pair(val[0], val[1]);
        double X = currenttype->as<ob::SE2StateSpace::StateType>(i)->getX();
        double Y = currenttype->as<ob::SE2StateSpace::StateType>(i)->getY();
        newCoords[i] = std::make_pair(X + val[0], Y + val[1]);
      }
      double D = 0;
      for (int i = 0; i < n - 1; ++i)
        for (int j = i + 1; j < n; ++j)
        {
          D = fmax(D, sqrt((newCoords[i].first - newCoords[j].first)*(newCoords[i].first - newCoords[j].first) +
                           (newCoords[i].second - newCoords[j].second)*(newCoords[i].second - newCoords[j].second)));
        }
      if (D < 1e-10)
        continue;
      if (D > equilibriumD)
        if (rng_.uniform01() < exp(1 - 5*D/equilibriumD))
          break;
      if (D < equilibriumD)
        //if (rng_.uniform01() < exp(1 - equilibriumD/D))
          break;
      if (fabs(D - equilibriumD) < 1e-10)
        break;
    }
    for (int i = 0; i < n; ++i)
    {
      statetype->as<ob::SE2StateSpace::StateType>(i)->setX(newCoords[i].first);
      statetype->as<ob::SE2StateSpace::StateType>(i)->setY(newCoords[i].second);
      statetype->as<ob::SE2StateSpace::StateType>(i)->setYaw(0.);
    }
  }

  virtual void sampleUniformFar(ob::State *state, const ob::State *current)
  {
    auto currenttype = static_cast<const ob::CompoundStateSpace::StateType*>(current);
    auto bounds = space_->as<ob::CompoundStateSpace>()->getSubspace(0)->as<ob::SE2StateSpace>()->getBounds();
    int n = space_->as<ob::CompoundStateSpace>()->getSubspaceCount();
    double dXplus = bounds.high[0]-bounds.low[0],
           dXminus = bounds.high[0]-bounds.low[0],
           dYplus = bounds.high[1]-bounds.low[1],
           dYminus = bounds.high[1]-bounds.low[1];
    for (int i = 0; i < n; ++i)
    {
      double X = currenttype->as<ob::SE2StateSpace::StateType>(i)->getX();
      double Y = currenttype->as<ob::SE2StateSpace::StateType>(i)->getY();
      dXplus = fmin(dXplus, bounds.high[0] - X);
      dXminus = fmin(dXminus, X - bounds.low[0]);
      dYplus = fmin(dYplus, bounds.high[1] - Y);
      dYminus = fmin(dYminus, Y - bounds.low[1]);
    }
    double dX = rng_.uniformReal(-dXminus, dXplus)*fractionFar;
    double dY = rng_.uniformReal(-dYminus, dYplus)*fractionFar;
    auto dXarray = new double[n];
    auto dYarray = new double[n];
    for (int i = 0; i < n; ++i) dXarray[i] = dX;
    for (int i = 0; i < n; ++i) dYarray[i] = dY;
    sampleTranslation(state, current, dXarray, dYarray);
    delete[] dXarray;
    delete[] dYarray;
  }

  void sampleTranslation(ob::State *state, const ob::State *current, double* dX, double* dY)
  {
    auto statetype = static_cast<ob::CompoundStateSpace::StateType*>(state);
    auto currenttype = static_cast<const ob::CompoundStateSpace::StateType*>(current);
    auto bounds = space_->as<ob::CompoundStateSpace>()->getSubspace(0)->as<ob::SE2StateSpace>()->getBounds();

    for (int i = 0; i < space_->as<ob::CompoundStateSpace>()->getSubspaceCount(); ++i)
    {
      double X = currenttype->as<ob::SE2StateSpace::StateType>(i)->getX();
      double Y = currenttype->as<ob::SE2StateSpace::StateType>(i)->getY();
      X += dX[i];
      X = fmax(X, bounds.low[0]);
      X = fmin(X, bounds.high[0]);
      statetype->as<ob::SE2StateSpace::StateType>(i)->setX(X);
      Y += dY[i];
      Y = fmax(Y, bounds.low[1]);
      Y = fmin(Y, bounds.high[1]);
      statetype->as<ob::SE2StateSpace::StateType>(i)->setY(Y);
      statetype->as<ob::SE2StateSpace::StateType>(i)->setYaw(0.);
    }
  }

  void sampleRandom(ob::State *state)
  {
    auto statetype = static_cast<ob::CompoundStateSpace::StateType*>(state);
    auto bounds = space_->as<ob::CompoundStateSpace>()->getSubspace(0)->as<ob::SE2StateSpace>()->getBounds();
    int n = space_->as<ob::CompoundStateSpace>()->getSubspaceCount();
    for (int i = 0; i < n; ++i)
    {
      double randX = rng_.uniformReal(bounds.low[0], bounds.high[0]);
      double randY = rng_.uniformReal(bounds.low[1], bounds.high[1]);
      statetype->as<ob::SE2StateSpace::StateType>(i)->setX(randX);
      statetype->as<ob::SE2StateSpace::StateType>(i)->setY(randY);
      statetype->as<ob::SE2StateSpace::StateType>(i)->setYaw(0.);
    }
  }

  void sampleTranslateInStateDirection(ob::State *directionState, ob::State *current, const double distance)
  {
    auto currenttype = static_cast<ob::CompoundStateSpace::StateType*>(current);
    auto directiontype = static_cast<ob::CompoundStateSpace::StateType*>(directionState);
    int n = space_->as<ob::CompoundStateSpace>()->getSubspaceCount();
    auto dX = new double[n];
    auto dY = new double[n];
    for (int i = 0; i < n; ++i)
    {
      double dirX = directiontype->as<ob::SE2StateSpace::StateType>(i)->getX();
      double dirY = directiontype->as<ob::SE2StateSpace::StateType>(i)->getY();
      double X = currenttype->as<ob::SE2StateSpace::StateType>(i)->getX();
      double Y = currenttype->as<ob::SE2StateSpace::StateType>(i)->getY();
      double scale = sqrt(pow(dirX - X, 2) + pow(dirY - Y, 2)) / distance;
      dX[i] = (dirX - X) / scale;
      dY[i] = (dirY - Y) / scale;
    }
    sampleTranslation(current, current, dX, dY);
    delete[] dX;
    delete[] dY;
  }

  virtual void sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
  {
  }

  double getDiameter(const ob::State *state)
  {
    auto statetype = static_cast<const ob::CompoundStateSpace::StateType*>(state);
    int n = space_->as<ob::CompoundStateSpace>()->getSubspaceCount();
    double D = 0;
    for (int i = 0; i < n - 1; ++i)
    {
      double Xi = statetype->as<ob::SE2StateSpace::StateType>(i)->getX();
      double Yi = statetype->as<ob::SE2StateSpace::StateType>(i)->getY();
      for (int j = i + 1; j < n; ++j)
      {
        double Xj = statetype->as<ob::SE2StateSpace::StateType>(j)->getX();
        double Yj = statetype->as<ob::SE2StateSpace::StateType>(j)->getY();
        D = fmax(D, sqrt((Xi - Xj)*(Xi - Xj) +
                         (Yi - Yj)*(Yi - Yj)));
      }
    }
    return D;
  }

  void setFractionFar(double f) { fractionFar = f; }
  void setFractionNear(double f) { fractionNear = f; }
  void setEquilibriumDiameter(double d) { equilibriumD = d; }
  void setEquilibriumDiameter(const ob::State *state)
  {
    equilibriumD = fmax(equilibriumD, getDiameter(state));
  }

private:
    double fractionFar, fractionNear;
    double equilibriumD;
};
