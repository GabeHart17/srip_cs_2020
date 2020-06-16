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
  ConfigurationSpace<DIMS>* csp;
  std::vector<PathTree<DIMS>* > goal_nodes;  // nodes that have reached the goal
  Point<DIMS> goal;
  double goal_radius;  // radius around which a point is considered to have reached the goal

  double shrinking_ball_radius(double eta) const {
    double gamma = pow(2, DIMS) * (1 + 1 / DIMS) * csp->lebesgue();
    double res = pow(gamma * (log(tree_size) / tree_size), 1 / DIMS);
    return fmin(res, eta);
  }

  static Point<DIMS> steer_(const Point<DIMS>& origin, const Point<DIMS>& target, double eta) {
    double s = eta / dist(origin, target);
    Point<DIMS> res = {};
    for (size_t i = 0; i < DIMS; i++) {
      res[i] = s * (target[i] - origin[i]) + origin[i];
    }
    return res;
  }
  void explore_(double eta) {
    Point<DIMS> sample = csp->random();
    PathTree<DIMS>* p_new = new PathTree<DIMS>(steer_(kdt.nearest(sample).point, sample, eta));
    PathTree<DIMS>* best = nullptr;
    double best_cost = INFINITY;
    std::vector<PathTree<DIMS>* > near = kdt.near(p_new->point, shrinking_ball_radius(eta));
    for (PathTree<DIMS>* p : near) {
      if (csp->is_unobstructed(p_new->point, p->point) &&
          p->cost + dist(p_new->point, p->point) < best_cost) {
        best = p;
        best_cost = p->cost + dist(p_new->point, p->point);
      }
    }
    if (best != nullptr) {
      p_new->cost = best_cost;
      p_new->parent = best;
      best->children.push_back(p_new);
      tree_size++;
      if (dist(p_new->point, goal) < goal_radius) {
        goal_nodes.push_back(p_new);
      }
      rewire_neighbors_(*p_new, near);
    }
  }

  void rewire_neighbors_(PathTree<DIMS>& p_new, std::vector<PathTree<DIMS>* >& neighbors) {
    for (PathTree<DIMS>* p : neighbors) {
      if (p != p_new.parent) {
        double c = dist(p_new.point, p->point);
        if (p_new.cost + c < p->cost && csp->is_unobstructed(p_new.point, p->point)) {
          for (size_t i = 0; i < p->parent->children.size(); i++) {
            if (p->parent->children[i] == p) {
              p->parent->children.erase(p->parent->children.begin() + i);
              break;
            }
          }
          p->parent = &p_new;
          p_new.children.push_back(p);
          p->cost = p_new.cost + c;
        }
      }
    }
  }

  void build_tree_(unsigned int iterations, double eta) {
    for (unsigned int i = 0; i < iterations; i++) {
      explore_(eta);
    }
  }

public:
  RRTStarPlanner(ConfigurationSpace<DIMS>* c): csp(c), pt(nullptr) {}
  ~RRTStarPlanner() {
    delete pt;
  }

  std::vector<Point<DIMS> > path(Point<DIMS> start_pt, Point<DIMS> goal_pt,
                                 unsigned int iterations, double goal_radius, double eta) {
    tree_size = 0;
    goal_nodes.clear();
    if (pt != nullptr) delete pt;
    pt = new PathTree<DIMS>(start_pt);
    KDTree<DIMS> new_kdt(pt);
    kdt = new_kdt;
    build_tree_(iterations, eta);
    double best_cost = INFINITY;
    PathTree<DIMS>* best = nullptr;
    std::vector<Point<DIMS> > r_res;
    if (goal_nodes.empty()) return r_res;
    for (PathTree<DIMS>* p : goal_nodes) {
      if (p->cost < best_cost) {
        best_cost = p->cost;
        best = p;
      }
    }
    while (best != nullptr) {
      r_res.push_back(best->point);
      best = best->parent;
    }
    std::vector<Point<DIMS> > res;
    for (typename std::vector<Point<DIMS> >::reverse_iterator i = r_res.rbegin(); i != r_res.rend(); i++) {
      res.push_back(*i);
    }
    return res;
  }
};

#endif
