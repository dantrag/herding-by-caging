#ifndef HERDINGPLANNING_H
#define HERDINGPLANNING_H

#include <omplapp/config.h>
#include <omplapp/apps/AppBase.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

using namespace ompl;
using namespace ompl::app;

class HerdingPlanning : public AppBase<AppType::GEOMETRIC>
  {
  public:
    HerdingPlanning(unsigned int n);

    virtual ~HerdingPlanning(void) {}

    virtual base::ScopedState<> getDefaultStartState(void) const;

    virtual void inferEnvironmentBounds(void);

    virtual void inferProblemDefinitionBounds(void);

    bool isSelfCollisionEnabled(void) const
    {
      return true;
    }

    virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
    {
      return state;
    }

    virtual const base::StateSpacePtr& getGeometricComponentStateSpace(unsigned int index) const
    {
      return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(index);
    }

    virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
    {
      return getGeometricComponentStateSpace(0);
    }

    virtual unsigned int getRobotCount(void) const
    {
      return n_;
    }

  protected:
    virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const;

    unsigned int n_;
};

#endif // HERDINGPLANNING_H
