from rrt_star import RRTStar, TreeNode
import random


rrts = RRTStar()
root = TreeNode([8.0, 8.0])
rrts.tree.append(root)
rrts.tree.append(TreeNode([4.0, 5.0]))
rrts.tree.append(TreeNode([9.0, 6.0]))
rrts.tree.append(TreeNode([9.0, 5.0]))
rrts.tree.append(TreeNode([11.0, 7.0]))
rrts.kd_insert(1);
rrts.kd_insert(2);
rrts.kd_insert(3);
rrts.kd_insert(4);
print(rrts.nearest([9.0, 5.1]))
print(rrts.near([9, 5.5], 2.0))
print()
for i in rrts.tree:
    print(i.point, i.kd_parent, i.kd_smaller, i.kd_greater)
