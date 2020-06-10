#ifndef RRT_STAR_HEADER
#define RRT_STAR_HEADER

#include <vector>
#include "kd_tree.hpp"
#include "configuration_space.hpp"


template<size_t DIMS>
class RRTStarPlanner {
private:
  PathTree* pt;
  KDTree kdt;
  ConfigurationSpace csp;
  void explore_();
  void build_tree_(size_t);
public:
  RRTStarPlanner(ConfigurationSpace c): csp(c) {}
  ~RRTStarPlanner() {
    delete pt;
  }
  std::vector<Point<DIMS> > path(Point<DIMS>, Point<DIMS>, double);
};

#endif
