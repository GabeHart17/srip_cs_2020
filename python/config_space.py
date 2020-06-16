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

    def intersects(seg0s, seg0e, seg1s, seg1e):
        points = [seg0s, seg0e, seg1s, seg1e]
        for i in range(4):
            angle0 = math.atan2(points[i-1][0] - points[i][0], points[i-1][1] - points[i][1])
            angle1 = math.atan2(points[(i+1)%4][0] - points[i][0], points[(i+1)%4][1] - points[i][1])
            if abs(angle0 - angle1) > math.pi:
                return True
        return False

    def random(self):
        return (random.uniform(self.bounds[0], self.bounds[2]),
                random.uniform(self.bounds[1], self.bounds[3]))

    def is_free(self, point):
        return (ConfigurationSpace.contains(self.bounds, point) and not
                any([ConfigurationSpace.contains(i, point) for i in self.obstacles]))

    def is_unobstructed(self, start, end):
        if not self.is_free(start) or not self.is_free(end):
            return False
        for obs in self.obstacles:
            vertices = [[obs[0], obs[1]], [obs[2], obs[1]], [obs[2], obs[3]], [obs[0], obs[3]]]
            for i in range(4):
                if ConfigurationSpace.intersects(start, end, vertices[i], vertices[i-1]):
                    return False
        return True
