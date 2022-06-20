#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/StateSampler.h>
#include <functional>
#include <queue>

class RRTNoInterpolation : public ompl::geometric::RRT
{
public:

  RRTNoInterpolation(const ompl::base::SpaceInformationPtr &si);
  virtual ~RRTNoInterpolation();

	virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

	void setEpsilonDensity(int n) { eps_samples = n; }

  void setConfigRegistrator(std::function<void(ompl::base::State*)> reg);

  void setGoalRegionRect(double x1, double y1, double x2, double y2);

  void setFractionFar(double f) { fractionFar = f; }

  void setFractionNear(double f) { fractionNear = f; }

protected:
  void freeMemory();

  struct Config
  {
    ompl::geometric::RRT::Motion* motion;
    double eps;
    double weight;
    double prefixSumWeight;
  };

  std::queue<double> stateToCoordsQueue(ompl::base::State *state);
  double getEpsilon(ompl::base::State* state);
  double getMaxEdge(ompl::base::State *state);
  double getCycleSize(ompl::base::State *state);
  void getMassCenter(ompl::base::State *state, double &x, double &y);
  bool checkTranslateValidity(ompl::base::State *source, ompl::base::State *destination);
  int findConfigByRandomSeed(double w, int left, int right);
  int findConfigByRandomSeed(double w);
  void addConfig(ompl::geometric::RRT::Motion* motion);
  Config pickConfig();

  bool goalRegionIsSatisfied(ompl::base::State* state);

  double fractionFar, fractionNear;
  double goalX1, goalX2, goalY1, goalY2;
  std::function<void(ompl::base::State*)> reg_;
	int eps_samples;
  std::vector<Config> configurations;
  double totalWeight;
};
