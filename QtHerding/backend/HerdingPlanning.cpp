#include "HerdingPlanning.h"
#include <omplapp/config.h>
#include <omplapp/apps/AppBase.h>
#include <ompl/base/spaces/SE2StateSpace.h>

HerdingPlanning::HerdingPlanning(unsigned int n) :
  AppBase<ompl::app::AppType::GEOMETRIC>(base::StateSpacePtr(new base::CompoundStateSpace()), Motion_2D), n_(n)
{
  assert (n > 0);
  name_ = "Herding planning (2D)";
  for (unsigned int i = 0; i < n_; ++i)
  {
    si_->getStateSpace()->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::SE2StateSpace()), 1.0);
  }
}

void HerdingPlanning::inferEnvironmentBounds(void)
{
  for (unsigned int i = 0; i < n_; ++i)
  {
    InferEnvironmentBounds(getGeometricComponentStateSpace(i), *static_cast<RigidBodyGeometry*>(this));
  }
}

void HerdingPlanning::inferProblemDefinitionBounds(void)
{
  for (unsigned int i = 0; i < n_; ++i)
  {
    InferProblemDefinitionBounds(AppTypeSelector<ompl::app::AppType::GEOMETRIC>::SimpleSetup::getProblemDefinition(),
      getGeometricStateExtractor(), factor_, add_,
      n_, getGeometricComponentStateSpace(i), mtype_);
  }
}

ompl::base::ScopedState<> HerdingPlanning::getDefaultStartState(void) const
{
  base::ScopedState<> st(getStateSpace());
  base::CompoundStateSpace::StateType* c_st = st.get()->as<base::CompoundStateSpace::StateType>();
  for (unsigned int i = 0; i < n_; ++i)
  {
    aiVector3D s = getRobotCenter(i);
    base::SE2StateSpace::StateType* sub = c_st->as<base::SE2StateSpace::StateType>(i);
    sub->setX(s.x);
    sub->setY(s.y);
    sub->setYaw(0.0);
  }
  return st;
}

const ompl::base::State* HerdingPlanning::getGeometricComponentStateInternal(const ompl::base::State* state, unsigned int index) const
{
  assert (index < n_);
  const base::SE2StateSpace::StateType* st = state->as<base::CompoundStateSpace::StateType>()->as<base::SE2StateSpace::StateType>(index);
  return static_cast<const base::State*>(st);
}
