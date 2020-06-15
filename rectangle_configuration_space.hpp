#ifndef RECTANGLE_CONFIGURATION_SPACE_HEADER
#define RECTANGLE_CONFIGURATION_SPACE_HEADER

#include <cmath>
#include <vector>
#include "configuration_space.hpp"

#define _USE_MATH_DEFINES


class Rectangle {
private:
  // the angle in radians to p2 from p1
  static double angle_to_(const Point<2>& p2, const Point<2>& p1) {
    return atan2(p2[0] - p1[0], p2[1] - p1[1]);
  }

  // returns true if segment l1 intersects segment l2
  static bool intersects_(const Point<2>& l1start, const Point<2>& l1end,
                         const Point<2>& l2start, const Point<2>& l2end)  {
    return abs(angle_to_(l1start, l2start) - angle_to_(l1start, l2end)) < M_PI &&
           abs(angle_to_(l2start, l1start) - angle_to_(l2start, l1end)) < M_PI &&
           abs(angle_to_(l1end, l2start) - angle_to_(l1end, l2end)) < M_PI &&
           abs(angle_to_(l2end, l1start) - angle_to_(l2end, l1end)) < M_PI;
  }

public:
  // diagonally opposite corners
  // chosen such that all coords of smaller are less than their counterpart in greater
  Point<2> smaller;
  Point<2> greater;
  double area;

  Rectangle(Point<2> s, Point<2> g): smaller(s), greater(g) {
    area = (greater[0] - smaller[0]) * (greater[1] - smaller[1]);
  }

  // whether a point falls within the rectangle
  bool contains(const Point<2>& p) const {
    return p[0] > smaller[0] && p[0] < greater[0] && p[1] > smaller[1] && p[1] < greater[1];
  }

  // whether a segment intersects or is within the rectangle
  bool obstructs(const Point<2>& p1, const Point<2>& p2) const {
    Point<2> gs = {greater[0], smaller[1]};
    Point<2> sg = {smaller[0], greater[1]};
    return contains(p1) || contains(p2) ||
           intersects_(p1, p2, smaller, sg) ||
           intersects_(p1, p2, sg, greater) ||
           intersects_(p1, p2, greater, gs) ||
           intersects_(p1, p2, gs, smaller);
  }
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

  // true if the segment between p0 and p1 does not intersect any obstructions
  bool is_unobstructed(const Point<2>& p0, const Point<2>& p1) const override {
    for (Rectangle r : obstacles) {
      if (r.obstructs(p0, p1)) return false;
    }
    return space.contains(p0) && space.contains(p1);
  }

  Point<2> random() const {
    Point<2> p = {0, 0};
    return p;
  }  // placeholder

  double lebesgue() const override { return free_area; }
};


#endif
