import random
import math
from conf.settings import RobotSettings


class Sampler(object):

    def __init__(self, lengths):
        self.lengths = lengths

    def sample_cart(self, n, x_min, y_min, x_max, y_max, theta_min, theta_max):
        """
        Uniform random sampling of a configuration of 2D points.
        :param n: Number of 2D points in a configuration
        :param x_min: The minimum x value in C-space
        :param y_min: The minimum y value in C-space
        :param x_max: The maximum x value in C-space
        :param y_max: The maximum y value in C-space
        :param theta_min: The minimum joint angle in C-space
        :param theta_max: The maximum joint angle in C-space
        :return: a list of coordinate tuples and a list of angles, as a tuple
        """
        # Generate 2 random numbers within the bounds as first point
        coordinates = [(0.0, -RobotSettings.BASE_Y_OFFSET)]
        # Once the first point is sampled, subsequent points are constrained
        # by the link length, so sample angles
        thetas = []
        if n > 1:
            thetas.append(random.uniform(theta_min, theta_max))
            phi = 0
            for i in range(n-1):
                phi += thetas[i]
                coordinates.append((coordinates[i][0] +
                                    self.lengths[i] * math.cos(
                                        math.radians(phi)),
                                    coordinates[i][1] +
                                    self.lengths[i] * math.sin(
                                        math.radians(phi))))
                if i < n-2:
                    thetas.append(random.uniform(theta_min, theta_max))
        return coordinates, thetas
