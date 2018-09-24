from conf.settings import *
from env.geometry import *


class Validator(object):

    def __init__(self):
        self.enlarge_factor = 0.5
        self.max_clearance = 0.0
        self.bounds = (0.0, 0.0, 0.0, 0.0)
        self.update_parameters()

    def set_enlarge_factor(self, factor):
        self.enlarge_factor = factor
        self.update_parameters()

    def update_parameters(self):
        self.max_clearance = self.enlarge_factor * BlockSettings.PUCK_DIAMETER
        self.bounds = (- WorkspaceSettings.WIDTH / 2 + self.max_clearance,
                       self.max_clearance,
                       WorkspaceSettings.WIDTH / 2 - self.max_clearance,
                       WorkspaceSettings.HEIGHT - self.max_clearance)

    def is_valid_sample(self, sample, obstacles, debug=False):
        """
        Check if the sample is collision-free and within the workspace.
        :param obstacles: the obstacles to check collision with
        :param sample: the sample to check
        :param debug: useful printouts if True
        :return: True if the sample is valid
        """
        if debug:
            bounds = False
            collision = False
            if self.fits_bounds(sample):
                print("Fits bounds, ", sample)
                bounds = True
            else:
                print("Does not fit bounds, ", sample)
            if self.has_collision(sample, obstacles, debug=True):
                print("Collision, ", sample)
                collision = True
            else:
                print("No collision, ", sample)
            return bounds and not collision

        return self.fits_bounds(sample) \
            and (not self.has_collision(sample, obstacles))

    def has_collision(self, cfg, obstacles, debug=False):
        """
        Checks if the configuration collides with any of the obstacles.
        :param cfg: the configuration to check
        :param obstacles: list<Shape2D> the obstacles in the workspace
        :param debug: useful printouts if True
        :return: True if there is a collision
        """
        big_puck = Circle(cfg.get_positions()[-1], self.max_clearance)
        if len(obstacles) == 1:
            if big_puck.intersects(obstacles[0]):
                return True
        else:
            for o in obstacles:
                # Parse single obstacle as a one-element list for consistency
                if self.has_collision(cfg, [o]):
                    if debug:
                        print("Collided with " + o.__str__())
                    return True
        return False

    def fits_bounds(self, cfg):
        """
        Determines whether the configuration's end effector position, i.e.
        the assumed puck position, lies within the workspace.
        :param cfg:
        :return:
        """
        end_pos = cfg.positions[-1]
        if end_pos[0] < self.bounds[0] \
                or end_pos[1] < self.bounds[1] \
                or end_pos[0] > self.bounds[2] \
                or end_pos[1] > self.bounds[3]:
            return False
        return True
