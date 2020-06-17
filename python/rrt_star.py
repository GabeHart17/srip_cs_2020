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

    def __eq__(self, other):
        return all([i == j for i, j in zip(self.point, other.point)])


class RRTStar:
    def __init__(self, configuration_space, eta):
        self.space = configuration_space
        self.tree = []
        self.goal_nodes = []
        self.goal = [0.0, 0.0]
        self.goal_radius = 1.0
        self.eta = eta  # step size

    def connect(self, parent, child):
        parent.children.append(child);
        child.parent = parent;

    def disconnect(self, parent, child):
        parent.children.remove(child)
        child.parent = None

    def distance(point1, point2):
        return math.hypot(point2[0] - point1[0], point2[1] - point1[1])

    def axis_distance(point1, point2, axis):
        return abs(point1[axis] - point2[axis])

    def steer(self, start, target):
        return [start[i] + (target[i] - start[i]) * (self.eta / RRTStar.distance(start, target)) for i in range(len(start))]

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

    def shrinking_ball_radius(self):
        gamma = 6.0 * self.space.lebesgue  # pow(2, DIMS) * (1 + 1 / DIMS) * csp->lebesgue()
        res = pow(gamma * (math.log(len(self.tree)) / len(self.tree)), 0.5)  # pow(gamma * (log(tree_size) / tree_size), 1 / DIMS)
        return min(res, self.eta)

    def cascade_cost(self, root):
        # print(self.tree.index(root))
        for c in root.children:
            c.cost = root.cost + RRTStar.distance(root.point, c.point)
            self.cascade_cost(c)

    def explore(self):
        r = self.space.random()
        nearest_neighbor = self.nearest(r)
        new_node = TreeNode(self.steer(nearest_neighbor.point, r))
        if not self.space.is_free(new_node.point):
            return
        best = None
        best_cost = math.inf
        neighbors = self.near(new_node.point, self.shrinking_ball_radius()) + [nearest_neighbor]
        for p in neighbors:
            if self.space.is_unobstructed(new_node.point, p.point):
                prospective_cost = p.cost + RRTStar.distance(new_node.point, p.point)
                if  prospective_cost < best_cost:
                    best = p
                    best_cost = prospective_cost
        if best is not None:
            new_node.cost = best_cost
            self.tree.append(new_node)
            self.connect(best, new_node)
            self.rewire_neighbors(new_node)
            self.kd_insert(new_node);  # insert after rewire, so new node doesn't come up in near search
            if (RRTStar.distance(new_node.point, self.goal) < self.goal_radius and
                self.space.is_unobstructed(new_node.point, self.goal)):
                self.goal_nodes.append(new_node)
            # print(new_node.parent.cost, new_node.cost)

    def is_ancestor(self, child, other):
        current = child
        while current is not None:
            if current == other:
                return True
            current = current.parent
        return False

    def rewire_neighbors(self, new_node):
        for n in self.near(new_node.point, self.shrinking_ball_radius()):
            prospective_cost = new_node.cost + RRTStar.distance(new_node.point, n.point)
            if prospective_cost < n.cost:
                self.disconnect(n.parent, n)
                self.connect(new_node, n)
                n.cost = prospective_cost
                self.cascade_cost(n)
                # print(new_node.cost < n.cost)

    def build_tree(self, start, goal, goal_radius, iterations):
        self.goal_nodes = []
        s = TreeNode(start)
        s.cost = 0.0
        self.tree = [s]
        self.goal = goal
        self.goal_radius = goal_radius
        for i in range(iterations):
            self.explore()

    def best_path(self):
        res = []
        current = min(self.goal_nodes, key=lambda x: x.cost)
        while current is not None:
            res.append(current.point)
            current = current.parent
        return list(reversed(res))
