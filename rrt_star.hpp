#ifndef RRT_STAR_HEADER
#define RRT_STAR_HEADER

#include <vector>
#include <cmath>
#include "kd_tree.hpp"
#include "configuration_space.hpp"


template<size_t DIMS>
class RRTStarPlanner {
private:
  unsigned int tree_size;
  PathTree<DIMS>* pt;
  KDTree<DIMS> kdt;
  ConfigurationSpace<DIMS> csp;

  double shrinking_ball_radius(double eta) const {
    double gamma = pow(2, DIMS) * (1 + 1 / DIMS) * csp.lebesgue();
    double res = pow(gamma * (log(tree_size) / tree_size), 1 / DIMS);
    return min(res, eta);
  }

  Point<DIMS> steer_(const Point<DIMS>& origin, const Point<DIMS>& target, double delta) {
    double s = delta / dist(origin, target);
    Point<DIMS> res = {};
    for (size_t i = 0; i < DIMS; i++) {
      res[i] = s * (target[i] - origin[i]);
    }
    return res;
  }

  void explore_(double delta) {
    Point<DIMS> sample = csp.random();
    PathTree<DIMS>* p_new = new PathTree<DIMS>(steer_(kdt.nearest(sample).point, delta));
    PathTree<DIMS>* best = nullptr;
    double best_cost = INFINITY;
    std::vector<PathTree<DIMS>& > near = kdt.near(p_new->point, shrinking_ball_radius(delta));
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
      tree_size++;
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

  void build_tree_(unsigned int iterations, double delta) {
    for (unsigned int i = 0; i < iterations; i++) {
      explore_(delta);
    }
  }

public:
  RRTStarPlanner(ConfigurationSpace<DIMS> c): csp(c), tree_size(0),  {}
  ~RRTStarPlanner() {
    delete pt;
  }
  std::vector<Point<DIMS> > path(const Point<DIMS>&, const Point<DIMS>&, double);
};

#endif
