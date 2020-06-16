import math


class TreeNode:
    def __init__(self, point):
        self.point = point
        self.kd_parent = None
        self.kd_greater = None
        self.kd_smaller = None
        self.parent = None
        self.children = []
        self.valid = True
        self.cost = math.inf

    def __str__(self):
        return self.point.__str__();


class RRTStar:
    def __init__(self, configuration_space=None):
        self.space = configuration_space
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

    def kd_insert(self, node):
        parent = self.tree[0]
        axis = 0
        while True:
            is_smaller = node.point[axis] < parent.point[axis]
            branch = parent.kd_smaller if is_smaller else parent.kd_greater
            if branch is None:
                if is_smaller:
                    parent.kd_smaller = node
                else:
                    parent.kd_greater = node
                node.kd_parent = parent
                break
            axis = (axis + 1) % 2
            parent = branch

    def nearest(self, point):
        best = self.tree[0]
        best_dist = RRTStar.distance(best.point, point)
        def nearest_helper(root, axis):
            nonlocal best, best_dist
            this_dist = RRTStar.distance(root.point, point)
            if this_dist < best_dist:
                best = root
                best_dist = this_dist
            smaller_side = point[axis] < root.point[axis]
            greater_side = not smaller_side
            both_sides = RRTStar.axis_distance(root.point, point, axis) < best_dist
            if (smaller_side or both_sides) and root.kd_smaller is not None:
                nearest_helper(root.kd_smaller, (axis + 1) % 2)
            if (greater_side or both_sides) and root.kd_greater is not None:
                nearest_helper(root.kd_greater, (axis + 1) % 2)
        nearest_helper(self.tree[0], 0)
        return best

    def near(self, point, radius):
        res = []
        def near_helper(root, axis):
            nonlocal res
            this_dist = RRTStar.distance(root.point, point)
            if this_dist < radius:
                res.append(root)
            smaller_side = point[axis] < root.point[axis]
            greater_side = not smaller_side
            both_sides = RRTStar.axis_distance(root.point, point, axis) < radius
            if (smaller_side or both_sides) and root.kd_smaller is not None:
                near_helper(root.kd_smaller, (axis + 1) % 2)
            if (greater_side or both_sides) and root.kd_greater is not None:
                near_helper(root.kd_greater, (axis + 1) % 2)
        near_helper(self.tree[0], 0)
        return res

    def shrinking_ball_radius(eta):
        gamma = 6 * self.space.lebesgue  # pow(2, DIMS) * (1 + 1 / DIMS) * csp->lebesgue()
        res = pow(gamma * (log(len(self.tree)) / len(self.tree)), 0.5)  # pow(gamma * (log(tree_size) / tree_size), 1 / DIMS)
        return min(res, eta)

    def explore(self):
        pass

    def rewire_neighbors(self, new_node):
        pass
