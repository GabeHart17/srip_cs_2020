#ifndef RRT_STAR_HEADER
#define RRT_STAR_HEADER

#include <vector>
#include <cmath>
#include "kd_tree.hpp"
#include "configuration_space.hpp"


template<size_t DIMS>
class RRTStarPlanner {
private:
  PathTree* pt;
  KDTree kdt;
  ConfigurationSpace csp;

  void explore_(double radius) {
    PathTree<DIMS>* p_new = new (csp.random());
    PathTree<DIMS>* best = nullptr;
    double best_cost = INFINITY;
    std::vector<PathTree<DIMS>& > near = kdt.near(p_new->point, radius);
    for (PathTree<DIMS>& p : near) {
      if (csp.is_unobstructed(p_new->point, p.point) &&
          p.cost + dist(p_new->point, p.point) < best_cost) {
        best = &p;
        best_cost = p.cost + dist(p_new->point, p.point);
      }
    }
    if (best != nullptr) {
      p_new->cost = best_cost;
      p_new->parent = best;
      best->children.push_back(p_new);
      rewire_neighbors_(*p_new, near);
    }
  }

  void rewire_neighbors_(PathTree<DIMS>& pt, std::vector<PathTree<DIMS>& >& neighbors);

  void build_tree_(unsigned int iterations);
public:
  RRTStarPlanner(ConfigurationSpace c): csp(c) {}
  ~RRTStarPlanner() {
    delete pt;
  }
  std::vector<Point<DIMS> > path(Point<DIMS>, Point<DIMS>, double);
};

#endif
