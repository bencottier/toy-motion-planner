import env.geometry as gm


class RobotConfig(object):

    def __init__(self, coordinates, angles):
        """
        Constructor
        :param coordinates: list of tuples of x,y coordinates
        :param angles: list of floats representing joint angles
        """
        self.positions = coordinates
        self.angles = angles
        self.connectors = {}

    def get_positions(self):
        return self.positions

    def get_position(self, i):
        return self.positions[i]

    def get_angles(self):
        return self.angles

    def max_distance(self, cfg_1):
        """
        Compute the maximum euclidean distance between corresponding
        positions of this configuration and the given configuration
        :param cfg_1:
        :return:
        """
        max_distance = 0.0
        for i in range(len(self.positions)):
            p0 = self.positions[i]
            p1 = cfg_1.positions[i]
            d = gm.distance(p0, p1)
            if d > max_distance:
                max_distance = d
        return max_distance

    def total_distance(self, cfg_1):
        total_distance = 0.0
        for i in range(len(self.positions)):
            total_distance += gm.distance(self.get_position(i),
                                          cfg_1.get_position(i))
        return total_distance

    def add_connector(self, v, e):
        self.connectors[v] = e

    def get_connectors(self):
        return self.connectors

    def get_cost_to_goal(self, goal):
        # TODO: heuristic
        return 0.0  # gm.distance(self.positions[-1], goal.positions[-1])

    def get_cost(self, v):
        return self.total_distance(v)

    def __repr__(self):
        return "\n({0}, {1})".format(str(self.positions), str(self.angles))


class PathJump(object):

    def __init__(self, u, v):
        self.vertices = [u, v]

    def get_vertices(self):
        return self.vertices
