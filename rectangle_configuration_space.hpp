#ifndef RECTANGLE_CONFIGURATION_SPACE
#define RECTANGLE_CONFIGURATION_SPACE

#include <cmath>
#include <vector>
#include "configuration_space.hpp"


class Rectangle {
public:
  // diagonally opposite corners
  // chosen such that all coords of smaller are less than their counterpart in greater
  Point<2> smaller;
  Point<2> greater;
  double area;

  Rectangle(Point<2> s, Point<2> g): smaller(s), greater(g) {
    area = (greater[0] - smaller[0]) * (greater[1] - smaller[1]);
  }

  bool contains(const Point<2>& p) const {
    return p[0] > smaller[0] && p[0] < greater[0] && p[1] > smaller[1] && p[1] < greater[1];
  }

  // bool intersects(const )
};


class RectangleConfigurationSpace : ConfigurationSpace<2> {
private:
  Rectangle space;
  std::vector<Rectangle> obstacles;
  double free_area;
public:
  RectangleConfigurationSpace(Rectangle s, std::vector<Rectangle> obs): space(s), obstacles(obs) {
    free_area = space.area;
    for (Rectangle r: obstacles) {
      free_area -= r.area;
    }
  }

  ~RectangleConfigurationSpace() override {}

  bool is_free(const Point<2>& p) const override {
    for (const Rectangle& obs : obstacles) {
      if (obs.contains(p)) return false;
    }
    return space.contains(p);
  }

  bool is_unobstructed(const Point<2>& p0, const Point<2>& p1) const override {
    if (!(is_free(p0) && is_free(p1))) return false;
    return true; // placeholder
  }

  virtual Point<2> random() const;  // random point in unobstructed space

  double lebesgue() const override { return free_area; }
};


#endif
