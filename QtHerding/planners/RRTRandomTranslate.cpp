#include "RRTRandomTranslate.h"
#include "backend/ashapes/ashapes2d.h"
#include "backend/HerdingSampler.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <memory>
#include <limits>
#include <cmath>

using namespace ompl;
using namespace ompl::geometric;

RRTRandomTranslate::RRTRandomTranslate(const base::SpaceInformationPtr &si) : RRT(si)
{
  eps_samples = 0;
}

void RRTRandomTranslate::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
    configurations.clear();
    pdef_->clearSolutionPaths();
}

RRTRandomTranslate::~RRTRandomTranslate()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nn_)
    nn_->clear();
  lastGoalMotion_ = nullptr;
}

std::queue<double> RRTRandomTranslate::stateToCoordsQueue(base::State *state)
{
  std::queue<double> coords;
  auto statetype = static_cast<ob::CompoundStateSpace::StateType*>(state);
  for (int i = 0; i < si_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspaceCount(); ++i)
  {
    coords.push(statetype->components[i]->as<ob::SE2StateSpace::StateType>()->getX());
    coords.push(statetype->components[i]->as<ob::SE2StateSpace::StateType>()->getY());
  }
  return coords;
}

double RRTRandomTranslate::getEpsilon(base::State *state)
{
  double eps, maxEdge;
  int cycle;
  runVerification(stateToCoordsQueue(state), eps, maxEdge, cycle);
  return eps;
}

double RRTRandomTranslate::getCycleSize(base::State *state)
{
  double eps, maxEdge;
  int cycle;
  runVerification(stateToCoordsQueue(state), eps, maxEdge, cycle);
  return cycle;
}

double RRTRandomTranslate::getMaxEdge(base::State *state)
{
  double eps, maxEdge;
  int cycle;
  runVerification(stateToCoordsQueue(state), eps, maxEdge, cycle);
  return maxEdge;
}

void RRTRandomTranslate::getMassCenter(base::State *state, double &x, double &y)
{
  x = 0.0;
  y = 0.0;
  auto statetype = static_cast<ob::CompoundStateSpace::StateType*>(state);
  int ncoords = si_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspaceCount();
  for (int i = 0; i < ncoords; ++i)
  {
    x += statetype->components[i]->as<ob::SE2StateSpace::StateType>()->getX();
    y += statetype->components[i]->as<ob::SE2StateSpace::StateType>()->getY();
  }
  x /= ncoords;
  y /= ncoords;
}

bool RRTRandomTranslate::checkTranslateValidity(base::State *source, base::State *destination)
{
  double xSource, ySource, xDest, yDest, translationX, translationY, xS, yS, xD, yD, moveX, moveY, distance, translationLength;
  double epsSource, epsDest, maxEdgeSource, maxEdgeDest;
  double epsCoef = 0.5;
  int cycleSource, cycleDest;
  int ncoords = si_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspaceCount();
  auto sourceStateType = static_cast<ob::CompoundStateSpace::StateType*>(source);
  auto destStateType = static_cast<ob::CompoundStateSpace::StateType*>(destination);
  bool valid;

  runVerification(stateToCoordsQueue(source), epsSource, maxEdgeSource, cycleSource);
  runVerification(stateToCoordsQueue(destination), epsDest, maxEdgeDest, cycleDest);

  getMassCenter(source, xSource, ySource);
  getMassCenter(destination, xDest, yDest);
  translationX = xDest - xSource;
  translationY = yDest - ySource;
  translationLength = sqrt(pow(translationX, 2) + pow(translationY, 2));
  valid = true;
  for (int i = 0; i < ncoords; ++i)
  {
    moveX = 0;
    moveY = 0;
    xS = sourceStateType->components[i]->as<ob::SE2StateSpace::StateType>()->getX();
    yS = sourceStateType->components[i]->as<ob::SE2StateSpace::StateType>()->getY();
    xD = destStateType->components[i]->as<ob::SE2StateSpace::StateType>()->getX();
    yD = destStateType->components[i]->as<ob::SE2StateSpace::StateType>()->getY();
    moveX = xS + translationX - xD;
    moveY = yS + translationY - yD;
    distance = sqrt(pow(moveX, 2) + pow(moveY, 2));
    if ((distance >= epsSource*epsCoef) || (distance >= epsDest*epsCoef)){
      valid = false;
      std::cout << xS  << " " << yS << std::endl;
      std::cout << xD  << " " << yD << std::endl;
      std::cout << distance << std::endl;
    }
  }

  std::cout << "valid: " << valid << std::endl;
  return valid;
}

void RRTRandomTranslate::setGoalRegionRect(double x1, double y1, double x2, double y2)
{
  goalX1 = fmin(x1, x2);
  goalX2 = fmax(x1, x2);
  goalY1 = fmin(y1, y2);
  goalY2 = fmax(y1, y2);
}

bool RRTRandomTranslate::goalRegionIsSatisfied(ompl::base::State* state)
{
  bool sat = true;
  auto statetype = static_cast<ob::CompoundStateSpace::StateType*>(state);
  for (int i = 0; i < si_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspaceCount(); ++i)
  {
    double X = statetype->components[i]->as<ob::SE2StateSpace::StateType>()->getX();
    double Y = statetype->components[i]->as<ob::SE2StateSpace::StateType>()->getY();
    if ((X > goalX2) || (X < goalX1) || (Y > goalY2) || (Y < goalY1))
    {
      sat = false;
      break;
    }
  }
  return sat;
}

void RRTRandomTranslate::setConfigRegistrator(std::function<void(ompl::base::State*, ompl::base::State*)> reg)
{
  reg_ = reg;
}

void RRTRandomTranslate::addConfig(ompl::geometric::RRT::Motion* motion)
{
  Config conf = {};
  conf.motion = motion;
  conf.eps = getEpsilon(motion->state);
  conf.weight = conf.eps * (1 + configurations.size()*configurations.size() / 10000);
  totalWeight += conf.weight;
  conf.prefixSumWeight = totalWeight;
  configurations.push_back(conf);
}

int RRTRandomTranslate::findConfigByRandomSeed(double w, int left, int right)
{
  if (left == right)
    return left;

  int mid = (left + right) / 2;
  if (w <= configurations[mid].prefixSumWeight)
    return findConfigByRandomSeed(w, left, mid);
  else
    return findConfigByRandomSeed(w, mid + 1, right);
}

int RRTRandomTranslate::findConfigByRandomSeed(double w)
{
  return findConfigByRandomSeed(w, 0, configurations.size() - 1);
}

RRTRandomTranslate::Config RRTRandomTranslate::pickConfig()
{
  RNG rng;
  double w = rng.uniform01() * totalWeight;
  return configurations[findConfigByRandomSeed(w)];
}

base::PlannerStatus RRTRandomTranslate::solve(const base::PlannerTerminationCondition &ptc)
{
  checkValidity();

  configurations.clear();
  totalWeight = 0.0;

  while (const base::State *st = pis_.nextStart())
  {
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    nn_->add(motion);
    reg_(motion->state, nullptr);
    addConfig(motion);
  }

  if (nn_->size() == 0)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  if (!sampler_) sampler_ = si_->allocStateSampler();

  auto sampler = std::dynamic_pointer_cast<HerdingSampler>(sampler_);
  sampler->setFractionFar(fractionFar);
  sampler->setFractionNear(fractionNear);
  pis_.restart();
  sampler->setEquilibriumDiameter(pis_.nextStart());

  OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

  Motion *solution  = nullptr;
  Motion *rmotion   = new Motion(si_);
  base::State *rstate = rmotion->state;
  base::State *xstate = si_->allocState();

  while (ptc == false)
  {
    sampler->sampleRandom(rstate);

    Motion *nmotion = nn_->nearest(rmotion);
    si_->copyState(xstate, nmotion->state);
    sampler->sampleTranslateInStateDirection(rstate, xstate, getEpsilon(xstate)/100);

    std::pair<base::State*, double> lastValid;
    lastValid.first = xstate;
    bool bInterpolated = false;
    if (!si_->checkMotion(nmotion->state, xstate, lastValid))
    {
      xstate = lastValid.first;
      bInterpolated = true;
    }
    if (!bInterpolated || (lastValid.second > 0))
    {
      // now this check is valid for both directions
      //if (checkTranslateValidity(nmotion->state, xstate))
      {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, xstate);
        motion->parent = nmotion;

        nn_->add(motion);
        addConfig(motion);
        reg_(xstate, nmotion->state);

        bool sat = goalRegionIsSatisfied(motion->state);
        if (sat)
        {
          solution = motion;
          break;
        }
      }
    }
  }

  bool solved = false;
  if (solution != nullptr)
  {
      lastGoalMotion_ = solution;

      std::vector<Motion*> mpath;
      while (solution != nullptr)
      {
          mpath.push_back(solution);
          solution = solution->parent;
      }

      PathGeometric *path = new PathGeometric(si_);
      for (int i = mpath.size() - 1 ; i >= 0 ; --i)
          path->append(mpath[i]->state);
      pdef_->addSolutionPath(base::PathPtr(path));
      solved = true;
  }

  si_->freeState(xstate);
  if (rmotion->state)
      si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

  return base::PlannerStatus(solved, false);
}

/*
16
16
28
28
22

-20
-35
-20
-29
-20
*/

/*
18
16
16
18
24
26
26
24

-30
-26
-22
-18
-30
-26
-22
-18
*/
