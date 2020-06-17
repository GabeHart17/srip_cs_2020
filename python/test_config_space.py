from config_space import ConfigurationSpace


bounds = [0.0, 0.0, 8.0, 16.0]
obstacles = [
            [1.0, 1.0, 3.0, 4.0]
            ]
space = ConfigurationSpace(bounds, obstacles)
p0 = [8.0, 8.0]
p1 = [4.0, 5.0]
p2 = [2.0, 3.0]
p3 = [0.5, 0.5]
p4 = [-1.0, 1.0]
print(space.is_free(p0), space.is_free(p1), space.is_free(p2), space.is_free(p4))
print(space.is_unobstructed(p0, p1))
print(space.is_unobstructed(p1, p2))
print(space.is_unobstructed(p1, p3))

# a = [0.0, 0.0]
# b = [0.0, 1.0]
# c = [1.0, 0.0]
# d = [1.0, 1.0]
# e = [0.1, 0.1]
# print(ConfigurationSpace.intersects(a, d, b, c))
# print(ConfigurationSpace.intersects(a, e, b, c))
# print(ConfigurationSpace.intersects(a, b, c, d))
