from config_space import ConfigurationSpace
from rrt_star import RRTStar
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PatchCollection
from matplotlib.patches import Rectangle, Circle


bounds = [0.0, 0.0, 8.0, 8.0]
obstacles = [
            [5.0, 5.0, 7.0, 7.0],
            [1.0, 0.0, 2.5, 5.0]
            ]
space = ConfigurationSpace(bounds, obstacles)
p0 = [0.5, 0.5]
p1 = [7.5, 6.0]
planner = RRTStar(space, 0.25)
planner.build_tree(p0, p1, 0.5, 10000)
# for i in planner.tree:
#     print(i, i.parent)
print(len(planner.goal_nodes))
print(len(planner.tree))
res = planner.best_path()
# for i in res:
#     print(i)
print(all([i.cost > i.parent.cost for i in planner.tree[1:]]))
# plt.scatter([i.point[0] for i in planner.tree], [i.point[1] for i in planner.tree], s=4)
# for i in planner.tree[1:]:
#     plt.plot([i.parent.point[0], i.point[0]], [i.parent.point[1], i.point[1]], 'b-', lw=1)
ax = plt.subplots()[1]
ax.plot([i[0] for i in res], [i[1] for i in res], 'm-', lw=4, zorder=1)
tree_col = LineCollection([(i.point, i.parent.point) for i in planner.tree[1:]],
                          linewidths=(1,),
                          colors=('b',),
                          zorder=1)
ax.add_collection(tree_col)
obs_rects = [Rectangle((i[0], i[1]), i[2]-i[0], i[3]-i[1]) for i in obstacles]
obs_col = PatchCollection(obs_rects, facecolors=('k',), zorder=0)
ax.add_collection(obs_col)
ax.add_patch(Circle(p1, 0.5, color='g', alpha=0.5, zorder=0))
plt.scatter([p0[0]], [p0[1]], c='r', s=100, zorder=2)
plt.scatter([p1[0]], [p1[1]], c='g', s=100, zorder=2)
plt.show()
