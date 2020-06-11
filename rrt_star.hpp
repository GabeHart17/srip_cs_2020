#ifndef RRT_STAR_HEADER
#define RRT_STAR_HEADER

#include <vector>
#include <cmath>
#include "kd_tree.hpp"
#include "configuration_space.hpp"


template<size_t DIMS>
class RRTStarPlanner {
private:
  PathTree<DIMS>* pt;
  KDTree<DIMS> kdt;
  ConfigurationSpace<DIMS> csp;

  Point<DIMS> steer_(const Point<DIMS>& origin, const Point<DIMS>& target, double r) {
    double s = r / dist(origin, target);
    Point<DIMS> res = {};
    for (size_t i = 0; i < DIMS; i++) {
      res[i] = s * (target[i] - origin[i]);
    }
    return res;
  }

  void explore_(double radius) {
    Point<DIMS> sample = csp.random();
    PathTree<DIMS>* p_new = new PathTree<DIMS>(steer_(kdt.nearest(sample).point, sample));
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

  void rewire_neighbors_(PathTree<DIMS>& p_new, std::vector<PathTree<DIMS>& >& neighbors) {
    for (PathTree<DIMS>& p : neighbors) {
      if (&p != p_new.parent) {
        double c = dist(p_new.point, p.point);
        if (p_new.cost + c < p.cost && csp.is_unobstructed(p_new.point, p.point)) {
          for (size_t i = 0; i < p.parent->children.size(); i++) {
            if (p.parent->children[i] == &p) {
              p.parent->children.erase(i);
              break;
            }
          }
          p.parent = &p_new;
          p_new.children.push_back(&p);
          p.cost = p_new.cost + c;
        }
      }
    }
  }

  void build_tree_(unsigned int iterations);

public:
  RRTStarPlanner(ConfigurationSpace<DIMS> c): csp(c) {}
  ~RRTStarPlanner() {
    delete pt;
  }
  std::vector<Point<DIMS> > path(const Point<DIMS>&, const Point<DIMS>&, double);
};

#endif
