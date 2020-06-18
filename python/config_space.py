import math
import random

# rectangles, such as the boundaries of the space and the obstacles are
# represented by sequences of 4 elements
# the first two represent the "smaller" corner coords, the second two the larger


class ConfigurationSpace:
    def __init__(self, bounds, obstacles):
        self.bounds = bounds
        self.obstacles = obstacles
        self.lebesgue = (bounds[2] - bounds[0]) * (bounds[3] - bounds[1])
        for obs in obstacles:
            self.lebesgue -= (obs[2] - obs[0]) * (obs[3] - obs[1])

    def contains(rect, point):
        return rect[0] <= point[0] <= rect[2] and rect[1] <= point[1] <= rect[3]

    def random(self):
        return (random.uniform(self.bounds[0], self.bounds[2]),
                random.uniform(self.bounds[1], self.bounds[3]))

    def is_free(self, point):
        return (ConfigurationSpace.contains(self.bounds, point) and not
                any([ConfigurationSpace.contains(i, point) for i in self.obstacles]))

    def is_unobstructed(self, start, end):
        is_vertical = start[0] == end[0]
        is_horizontal = start[1] == end[1]
        slope = (end[1] - start[1]) / (end[0] - start[0]) if not is_vertical else math.inf
        islope = (end[0] - start[0]) / (end[1] - start[1]) if not is_horizontal else math.inf
        def intersects_vertical(x, y0, y1):
            prediction = start[1] + (x - start[0]) * slope
            return y0 < prediction < y1 and min(start[0], end[0]) < x < max(start[0], end[0])
        def intersects_horizontal(y, x0, x1):
            prediction = start[0] + (y - start[1]) * islope
            return x0 < prediction < x1 and min(start[1], end[1]) < y < max(start[1], end[1])
        if not self.is_free(start) or not self.is_free(end):
            return False
        for obs in self.obstacles:
            if not is_vertical:
                if intersects_vertical(obs[0], obs[1], obs[3]) or intersects_vertical(obs[2], obs[1], obs[3]):
                    return False
            if not is_horizontal:
                if intersects_horizontal(obs[1], obs[0], obs[2]) or intersects_horizontal(obs[3], obs[0], obs[2]):
                    return False
        return True


class DynamicObstacle:
    def __init__(self, init_bounds, rate = 1.0):
        self.bounds = init_bounds
        self.target = [0.0, 0.0]
        self.reached_target = True
        self.rate = rate

    def __getitem__(self, i):  # allows use in place of a list or tuple as an obstacle
        return self.bounds[i]

    def center(self):
        return [(bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2]

    def update(self):
        if not self.reached_target:
            xr = target[0] - center[0]
            yr = target[1] - center[1]
            r = self.rate / math.hypot(xr, yr)
            self.bounds[0] += xr * r
            self.bounds[1] += yr * r
            self.bounds[2] += xr * r
            self.bounds[3] += yr * r
            if xr * (target[0] - center[0]) < 0 or yr * (target[1] < center[1]) < 0:
                self.reached_target = True
