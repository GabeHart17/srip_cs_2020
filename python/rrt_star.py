import math


class TreeNode:
    def __init__(self, point):
        self.point = point
        self.kd_parent = -1
        self.kd_greater = -1
        self.kd_smaller = -1
        self.parent = -1
        self.children = []
        self.valid = True
        self.cost = math.inf


class RRTStar:
    def __init__(self):
        self.tree = []
        self.goal_nodes = []
        self.goal = [0.0, 0.0]

    def connect(self, parent, child):
        self.tree[parent].children.append(child);
        self.tree[child].parent = parent;

    def disconnect(self, parent, child):
        self.tree[parent].children.remove(child)
        self.tree[child].parent = -1

    def distance(point1, point2):
        return math.hypot(point2[0] - point1[0], point2[1] - point1[1])

    def axis_distance(point1, point2, axis):
        return abs(point1[axis] - point2[axis])

    def steer(self, start, target, eta):
        return [start[i] + (target[i] - start[i]) * (eta / RRTStar.distance(start, target)) for i in range(len(start))]

    def kd_insert(self, index):
        parent = 0
        axis = 0
        while True:
            is_smaller = self.tree[index].point[axis] < self.tree[parent].point[axis]
            branch = self.tree[parent].kd_smaller if is_smaller else self.tree[parent].kd_greater
            if branch == -1:
                if is_smaller:
                    self.tree[parent].kd_smaller = index
                else:
                    self.tree[parent].kd_greater = index
                self.tree[index].kd_parent = parent
                break
            axis = (axis + 1) % 2
            parent = branch

    def nearest(self, point):
        best = 0
        best_dist = RRTStar.distance(self.tree[0].point, point)
        def nearest_helper(root, axis):
            nonlocal best, best_dist
            this_dist = RRTStar.distance(self.tree[root].point, point)
            if this_dist < best_dist:
                best = root
                best_dist = this_dist
            smaller_side = point[axis] < self.tree[root].point[axis]
            greater_side = not smaller_side
            both_sides = RRTStar.axis_distance(self.tree[root].point, point, axis) < best_dist
            if (smaller_side or both_sides) and self.tree[root].kd_smaller != -1:
                nearest_helper(self.tree[root].kd_smaller, (axis + 1) % 2)
            if (greater_side or both_sides) and self.tree[root].kd_greater != -1:
                nearest_helper(self.tree[root].kd_greater, (axis + 1) % 2)
        nearest_helper(0, 0)
        return best

    def near(self, point, radius):
        res = []
        def near_helper(root, axis):
            nonlocal res
            this_dist = RRTStar.distance(self.tree[root].point, point)
            if this_dist < radius:
                res.append(root)
            smaller_side = point[axis] < self.tree[root].point[axis]
            greater_side = not smaller_side
            both_sides = RRTStar.axis_distance(self.tree[root].point, point, axis) < radius
            if (smaller_side or both_sides) and self.tree[root].kd_smaller != -1:
                near_helper(self.tree[root].kd_smaller, (axis + 1) % 2)
            if (greater_side or both_sides) and self.tree[root].kd_greater != -1:
                near_helper(self.tree[root].kd_greater, (axis + 1) % 2)
        near_helper(0, 0)
        return res
