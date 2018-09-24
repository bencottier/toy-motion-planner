from __future__ import print_function

from env.scene import Scene
from env.path import Path
from env.super_path import SuperPath
from conf.settings import *
from .search.validator import Validator
from .search.astar import AStarSearch
from .search.robot_config import *
from .search.sampler import Sampler
from env.robot import Robot
import env.geometry as gm

import math
import os
import sys
import time
import numpy as np
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
from itertools import permutations

sys.path.append(os.path.abspath('..//'))


class Planner(object):
    LENGTHS = [RobotSettings.LINK_1_LENGTH, RobotSettings.LINK_2_LENGTH]
    # Maximum step size between configurations
    MAX_STEP = 2.0
    MAX_PROXIMITY = 60.0  # magic constant - how close to connect configs
    X_MAX = WorkspaceSettings.WIDTH
    Y_MAX = WorkspaceSettings.HEIGHT
    MAX_ANGLE = 230.0
    MIN_ANGLE = -50.0
    N = 3  # Number of joints (inc. end effector)
    TIMEOUT = 30  # seconds
    SAFE_FACTOR = 1.0
    RISKY_FACTOR = 0.6
    SWEAT_FACTOR = 0.5

    def __init__(self):
        self.scene = Scene()
        self.robot = Robot()
        self.obstacles = []
        self.path = Path()
        self.sampler = Sampler(self.LENGTHS)
        self.validator = Validator()
        self.configs = []
        self.path_found = False
        self.super_path = SuperPath()
        self.safety = True
        self.all_paths_found = False

    def received_scene_string(self, scene_string):
        """
        Convert the string to a scene, process the scene,
        and return a path string.
        """
        self.__init__()
        self.scene.decode(scene_string)
        print("Can Move: ", self.scene.moveableBlocks)
        self.scene.render()

        if len(self.scene.pucks) > len(self.scene.holes):
            print("{0} pucks, {1} goals - can't do it".format(len(self.scene.pucks), len(self.scene.holes)))
            exit()

        self.scene.print_scene()
        self.set_obstacles(self.scene, pucks=True)
        if len(self.scene.moveableBlocks) > 0:
            print("ATTEMPTING TO MOVE A BLOCK")
            self.move_block()
        # return self.super_path.to_string()

        all_paths, costs = self.find_all_paths()

        goal_indices, goal_costs = self.find_puck_goal_mapping(costs)

        for i in range(len(goal_indices)):
            self.super_path.paths.append(all_paths[i][goal_indices[i]])

        # Show path(s) in simulator
        sim_paths = [
            np.array([list(cfg.positions[-1]) for cfg in path.path])
            for path in self.super_path.paths
        ]
        # print("Planner: sim paths are \n", sim_paths)
        self.scene.render(paths=sim_paths)

        return self.super_path.to_string()

    def move_block(self):

        push_behind = 50
        push_distance = 50
        max_push_distance = 150

        # Plan path with movable block removed until path found
        for block in self.scene.moveableBlocks:
            self.set_obstacles(self.scene, pucks=True)
            if block.y < 0:
                continue
            print("Attempting to remove block at ", block.x, block.y)
            print("Length before removal = ", len(self.obstacles))
            self.obstacles.remove(block.shape)
            print("Length after removal = ", len(self.obstacles))

            if len(self.scene.moveableBlocks) == 1:
                all_paths, costs = self.find_all_paths(impatient=False)
            else:
                all_paths, costs = self.find_all_paths(impatient=True)

            if self.all_paths_found:
                self.configs = []
                block_to_move = block
                x = block.x
                y = block.y
                if block.blockType == BlockSettings.CYLINDER:
                    push_behind = 40
                if block.blockType == BlockSettings.CUBE:
                    push_behind = 45
                elif block.blockType == BlockSettings.PRISM:
                    push_behind = 65
                break

            self.configs = []

        if not self.all_paths_found:
            print("Error: Couldn't find a path for block pushing")
            exit()

        # Goals are obstacles when it comes to pushing
        self.set_obstacles(self.scene, pucks=True, goals=True)
        self.obstacles.remove(block_to_move.shape)

        min_d1 = 1000000
        min_d2 = 1000000
        nearest_block1 = None
        nearest_block2 = None
        for block in self.scene.blocks:
            if block.y < 0:
                continue
            d = gm.distance((block.x, block.y), (x, y))
            if d < 0.5:
                # THIS IS THE BLOCK THATS MOVING. 0.5 IS ARBITRARY
                continue
            if d < min_d1:
                min_d2 = min_d1
                min_d1 = d
                nearest_block2 = nearest_block1
                nearest_block1 = (block.x, block.y)
            elif d < min_d2:
                min_d2 = d
                nearest_block2 = (block.x, block.y)

        # Check if block is closer to boundary than one of the nearest
        ww = WorkspaceSettings.WIDTH
        wh = WorkspaceSettings.HEIGHT
        bottom = gm.Line2D((-ww / 2, 0.0), (ww / 2, 0.0))
        right = gm.Line2D((ww / 2, 0.0), (ww / 2, wh))
        top = gm.Line2D((ww / 2, wh), (-ww / 2, wh))
        left = gm.Line2D((-ww / 2, wh), (-ww / 2, 0.0))
        boundaries = [bottom, right, top, left]
        # Make the boundary point one of the nearest if so
        min_db = 1000000
        for boundary in boundaries:
            p = boundary.p_point((x, y))
            d = boundary.p_distance((x, y))
            if d < min_d1:
                min_d2 = min_d1
                min_d1 = d
                nearest_block2 = nearest_block1
                nearest_block1 = p
            elif d < min_d2:
                min_d2 = d
                nearest_block2 = p

        if nearest_block1 is None or nearest_block2 is None:
            # Maybe there were not enought blocks or something?
            # This shouldn't happen, but if it does it will cause problems
            print("Planner: Error finding nearest blocks in moving blocks")
            return

        print("Found Nearest Blocks: ", nearest_block1, nearest_block2)

        nearest_block_line = gm.Line2D(nearest_block1, nearest_block2)

        m1, c1 = nearest_block_line.find_normal_at_point((x, y))
        print("Nearest block line: m,c: ", m1, c1)

        # Direction of push
        theta = math.atan2(m1, 1.0)
        print("Theta is ", theta)
        print("x,y is :", x, y)

        start_point = gm.point_from_polar((x, y), push_behind, theta + np.pi)
        start_point = (start_point[0], start_point[1] - 5.0)
        print("Start point is: ", start_point)
        start_cfg = self.config_from_point(start_point)
        max_point = gm.point_from_polar((x, y), max_push_distance, theta)
        max_cfg = self.config_from_point(max_point)

        # Move forward (~ along path)
        # Check each direction until not in collision

        # TODO: intersect functions for prism and cube
        nearest_collision = self.step_to_block(start_cfg, max_cfg)

        if nearest_collision is None:
            push_distance = max_push_distance
        else:
            push_distance = gm.distance((x, y), nearest_collision)
        print("Nearest collision for first direction: ", nearest_collision)

        # See if opposite direction is better
        print("Checking opposite push direction")
        start_point2 = gm.point_from_polar((x, y), push_behind, theta)
        start_point2 = (start_point2[0], start_point2[1] - 5.0)
        start_cfg2 = self.config_from_point(start_point2)
        max_point2 = gm.point_from_polar((x, y), max_push_distance, theta + np.pi)
        max_cfg2 = self.config_from_point(max_point)
        nearest_collision2 = self.step_to_block(start_cfg2, max_cfg2)
        print("Nearest collision for second direction: ", nearest_collision2)

        if nearest_collision2 is None:
            if nearest_collision is not None:
                # Opposite direction does not collide -> better
                theta += np.pi
                start_point = start_point2
                start_cfg = start_cfg2
                push_distance = max_push_distance
        # Both None -> go with first option
        elif nearest_collision is not None:
            # Both have collision -> choose longest available push
            push_distance2 = gm.distance((x, y), nearest_collision2)
            if push_distance2 > push_distance:
                print("Chose opposite direction")
                # Direction 2 is better
                theta += np.pi
                start_point = start_point2
                start_cfg = start_cfg2
                push_distance = push_distance2
        # Else no change, go with direction 1

        end_point = gm.point_from_polar((x, y), push_distance - 20, theta)
        end_point = (end_point[0], end_point[1] - 5.0)
        print("End point is: ", end_point)
        end_cfg = self.config_from_point(end_point)

        path = Path()
        path.path.append(start_cfg)
        path.path.append(end_cfg)
        path.movingPath = True
        self.super_path.paths.append(path)

        # Update block position after push (inaccurate)
        self.set_obstacles(self.scene, pucks=True)
        self.obstacles.remove(block_to_move.shape)
        block_to_move.x = end_point[0]
        block_to_move.y = end_point[1]
        block_to_move.find_shape()
        print("Adding back the moved block: ", block_to_move.shape)
        # Remove goals for path planning
        self.obstacles.append(block_to_move.shape)

    def config_from_point(self, pt):
        angles = list(self.robot.inverse_kinematics(pt[0], pt[1], None, 0.0))
        coordinates = self.robot.forward_kinematics(angles[0], angles[1],
                                                    angles[2], all_joints=True)
        return RobotConfig(coordinates[:-1], angles[:-1])

    def set_obstacles(self, scene, pucks=False, goals=False):
        """
        Retrieves the obstacles in the environment based on the scene data.
        :param scene: the scene representing the state of the workspace.
        :return: list<Shape2D>
        """
        self.obstacles = [block.shape for block in scene.blocks]
        if pucks:
            self.obstacles.extend([puck.shape for puck in self.scene.pucks])
        if goals:
            self.obstacles.extend([hole.shape for hole in self.scene.holes])

    # print("Planner: scene is:\n", scene.print_scene())

    def find_all_paths(self, impatient=False):
        """
        Find all the paths from pucks to holes
        """
        # TODO handle invalid end states with super path
        # TODO iterate through all root-goal combos after each PRM build
        print("\nPlanner: planning safely ({0}x puck enlargement)"
              .format(2 * self.SAFE_FACTOR))

        if impatient:
            timeout = self.TIMEOUT / 2
        else:
            timeout = self.TIMEOUT

        # Build and search roadmap with random samples until all paths found
        # or we run out of time
        t0 = time.time()
        all_paths = [[None for h in range(len(self.scene.holes))]
                     for p in range(len(self.scene.pucks))]
        costs = [[None for h in range(len(self.scene.holes))]
                 for p in range(len(self.scene.pucks))]
        self.all_paths_found = False
        sample_size = 2
        while time.time() - t0 < timeout and not self.all_paths_found:
            print("\nPlanner: building roadmap with {0} samples"
                  .format(sample_size))

            if sample_size == 1024:
                # print("Planner: visualising")
                # self.visualise_graph()
                self.validator.set_enlarge_factor(self.RISKY_FACTOR)
                print("\nPlanner: living dangerously " \
                      "({0}x puck enlargement)".format(2 * self.RISKY_FACTOR))
                self.safety = False
            elif sample_size == 4096 and len(self.scene.moveableBlocks) == 0:
                self.validator.set_enlarge_factor(self.SWEAT_FACTOR)
                print("\nPlanner: start sweating ({0}x puck enlargement)"
                      .format(2 * self.SWEAT_FACTOR))

            self.build_road_map(sample_size)

            for root_index in range(len(self.scene.pucks)):
                # Reset to block obstacles only
                self.obstacles.remove(self.scene.pucks[root_index].shape)

                for goal_index in range(len(self.scene.holes)):
                    if all_paths[root_index][goal_index] is not None:
                        # Path already found; use that
                        continue
                    self.path = Path()
                    path_possible = self.do_query(root_index, goal_index)
                    if not path_possible:
                        continue
                    puck = self.scene.pucks[root_index]
                    hole = self.scene.holes[goal_index]
                    self.path.deltaTheta = puck.theta - hole.theta
                    # print("DELTA_ANGLE_IS: ", self.path.deltaTheta)
                    if self.path_found:
                        all_paths[root_index][goal_index] = self.path
                        costs[root_index][goal_index] = self.path.cost
                    self.path_found = False

                self.obstacles.append(self.scene.pucks[root_index].shape)

            if self.find_puck_goal_mapping(costs) is not None:
                self.all_paths_found = True

            # Extend the roadmap in the next iteration if paths were not found
            sample_size *= 2

        if time.time() - t0 > timeout:
            # TODO try again
            print("Planner: timeout at {0}s\n".format(time.time() - t0))
            return all_paths, costs

        if self.all_paths_found:
            print("Planner: found paths in {0}s\n".format(time.time() - t0))
            return all_paths, costs

    def do_query(self, root_index, goal_index):
        """
        Search for a path between the specified root and goal via the roadmap
        :return: True if a path was found
        """

        self.validator.set_enlarge_factor(0.5)
        print("Planner: finding path from puck {0} to goal {1}"
              .format(root_index, goal_index))

        root = self.get_end_state(0, index=root_index)
        goal = self.get_end_state(1, index=goal_index)
        if not self.validator.is_valid_sample(root, self.obstacles, debug=False) or \
                not self.validator.is_valid_sample(goal, self.obstacles, debug=False):
            print("Planner: ERROR: invalid end states")
            print("Valid root: ", self.validator.is_valid_sample(root, self.obstacles))
            print("Valid goal: ", self.validator.is_valid_sample(goal, self.obstacles))
            exit()

        if self.safety:
            self.validator.set_enlarge_factor(self.SAFE_FACTOR)
        else:
            self.validator.set_enlarge_factor(self.RISKY_FACTOR)

        # Try direct path
        direct_connection = self.make_steps(root, goal, check=True)[0]
        if direct_connection:
            self.path.set_path([root, goal])
            self.path_found = True
            self.path.cost = gm.distance(root.positions[-1],
                                         goal.positions[-1])
            print("Planner: goal found directly")
            return True

        # Find vertices nearest initial and goal configs
        connected_root = self.connect_configs(root)
        connected_goal = self.connect_configs(goal)

        # Check if root and goal were connected
        if not (connected_root and connected_goal):
            # Solution not possible
            return False

        # Initial and goal states are now in the roadmap
        # Search for a path
        ass = AStarSearch(root, goal)
        print("Planner: searching")
        ass.search()

        if ass.goal_found:
            # print("Planner: visualising")
            # self.visualise_graph()
            # Record solution
            path = ass.path
            full_path = []

            while len(path) > 1:
                cfg_0 = path.pop(0)
                # Uncomment below 4 lines to display/calculate curved paths
                # cfg_m = path[0]
                # steps = self.make_steps(cfg_0, cfg_m, check=False)[1]
                # steps.pop()
                # full_path += steps

                full_path.append(cfg_0)

            full_path.append(goal)
            self.path.set_path(full_path)
            self.path.cost = ass.total_cost
            self.path_found = True
            # Remove this root and goal from the roadmap, ready for the next
            for cfg in root.connectors:
                cfg.connectors.pop(root, False)
            for cfg in goal.connectors:
                cfg.connectors.pop(goal, False)
            return True

        return False

    def build_road_map(self, sample_size):
        """
        Add samples to the roadmap
        """
        while len(self.configs) < sample_size:
            self.sample_uniform()
        # print("Planner: Finished sampling, attempting to connect configs")
        self.connect_configs()

    def sample_uniform(self):
        """
        Sample a configuration uniformly at random.
        :return: True if the configuration was connected to any others
        """
        sample = self.sampler.sample_cart(self.N, 0.0, 0.0,
                                          self.X_MAX, self.Y_MAX,
                                          self.MIN_ANGLE, self.MAX_ANGLE)
        cfg = RobotConfig(sample[0], sample[1])
        if self.validator.is_valid_sample(cfg, self.obstacles):
            self.configs.append(cfg)

    def connect_configs(self, cfg_0=None):
        """
        Connect a new configuration to a maximum of k others in the state
        graph, if they are within a certain distance and there is a
        collision-free path between them.
        :param cfg_0: The new configuration to connect to
        :return: True if any connections were made
        """
        num_connections = 0
        if len(self.configs) == 0:
            return False
        elif len(self.configs) < 5:
            n_neighbors = len(self.configs)
        else:
            n_neighbors = 5
        # Make array of end effector positions
        # print("Planner: Building points")
        points = np.array([cfg.positions[-1] for cfg in self.configs])
        # print("Planner: Finding NearestNeighbors")
        neigh = NearestNeighbors(n_neighbors=n_neighbors,
                                 radius=self.MAX_PROXIMITY)
        # print("Planner: Fitting points")
        neigh.fit(points)
        # distances, indices = neigh.radius_neighbors([cfg_0.positions[-1]])
        if cfg_0 is not None:
            distances, indices = neigh.kneighbors([cfg_0.positions[-1]])
            for i in indices[0]:
                cfg_m = self.configs[i]
                # if cfg_m not in cfg_0.connectors.keys():
                num_connections += self.make_steps(cfg_0, cfg_m)[0]
            return num_connections > 0
        else:
            distances, indices = neigh.kneighbors(points)
            for i in indices:
                cfg_0 = self.configs[i[0]]
                for j in i[1:]:
                    cfg_m = self.configs[j]
                    if cfg_m not in cfg_0.connectors.keys():
                        self.make_steps(cfg_0, cfg_m)
            return False

    def make_steps(self, cfg_0, cfg_m, check=True):
        """
        Interpolate a straight-line path between configurations in C-space.
        :param cfg_0: the initial configuration
        :param cfg_m: the final configuration
        :param check: check if the path is collision free if True
        :return: a tuple; the first element is 1 if a connection is
        made and 0 otherwise; the second element is the interpolated
        configurations from the initial to the final state inclusive.
        """
        connection = 0  # 1 if valid connection is made
        steps = [cfg_0, cfg_m]
        end_loop = False
        max_step = 1000.0  # max of all iterations

        while True:
            old_steps = list(steps)
            current_max_step = 0  # max of current iteration
            end_index = len(steps) - 1
            for i in range(end_index):
                # Interpolate the midpoint configuration
                cfg_1 = old_steps[i]
                cfg_2 = old_steps[i + 1]
                cfg_nxt = self.midpoint(cfg_1, cfg_2)
                steps.insert(2 * i + 1, cfg_nxt)
                i_max_step = gm.distance(cfg_1.positions[-1],
                                         cfg_nxt.positions[-1])

                if i_max_step > current_max_step:
                    current_max_step = i_max_step

                if check and (self.validator.has_collision(cfg_nxt,
                                                           self.obstacles)
                              or not self.validator.fits_bounds(cfg_nxt)):
                    end_loop = True
                    break

            if current_max_step < max_step:
                max_step = current_max_step

            if max_step < self.MAX_STEP:
                # No collision and primitive step size has been reached
                if check:
                    # Connect vertices in state graph
                    jump = PathJump(cfg_0, cfg_m)
                    cfg_0.add_connector(cfg_m, jump)
                    cfg_m.add_connector(cfg_0, jump)
                    connection = 1
                break
            if end_loop:
                break
        return connection, steps

    def step_to_block(self, cfg_0, cfg_m):

        steps = [cfg_0, cfg_m]

        p0 = cfg_0.positions[-1]
        pm = cfg_m.positions[-1]
        d = gm.distance(p0, pm)
        # print(d)
        # print(p0, pm)
        theta = math.atan2(float(pm[1] - p0[1]), (pm[0] - p0[0]))
        # print(theta)
        num_steps = int(math.ceil(d * 1.0 / self.MAX_STEP))
        # print(num_steps)

        while len(steps) - 1 < num_steps:
            for i in range(num_steps - 1):
                cfg_1 = steps[i]
                cfg_nxt = self.config_from_point(gm.point_from_polar(
                    cfg_1.positions[-1], self.MAX_STEP, theta))
                steps.insert(i + 1, cfg_nxt)

                if (not self.validator.fits_bounds(cfg_nxt)) \
                        or self.validator.has_collision(
                        cfg_nxt, self.obstacles):
                    print(i)
                    print("hit block or out of bounds")
                    print(cfg_nxt.positions[-1])
                    return cfg_nxt.positions[-1]

        # print(steps)
        # print(len(steps))
        return None

    def midpoint(self, cfg_0, cfg_1):
        coordinates = []
        angles = []
        # p00 = cfg_0.get_positions()[0]
        # p10 = cfg_1.get_positions()[0]
        # p_mid_0 = ((p00[0] + p10[0])*1.0/2, (p00[1] + p10[1])*1.0/2)
        # Assume same base position
        coordinates.append(cfg_0.get_position(0))
        thetas_0 = cfg_0.get_angles()
        thetas_1 = cfg_1.get_angles()
        delta_thetas = self.delta_thetas(thetas_0, thetas_1)
        phi = 0
        for i in range(len(thetas_0)):
            p_ref = coordinates[i]
            # Rotation = preceding angle for mid config + this angle for
            # previous config + change in this angle
            theta_mid = thetas_0[i] + delta_thetas[i] * 1.0 / 2
            angles.append(theta_mid)
            phi += theta_mid
            p_mid = (p_ref[0] + self.LENGTHS[i] * math.cos(math.radians(phi)),
                     p_ref[1] + self.LENGTHS[i] * math.sin(math.radians(phi)))
            coordinates.append(p_mid)
        return RobotConfig(coordinates, angles)

    def delta_thetas(self, thetas_0, thetas_1):
        delta_thetas = []
        for i in range(self.N - 1):
            delta_theta = thetas_1[i] - thetas_0[i]
            if i == 0:
                delta_theta = self.normalise_angle(delta_theta)
            delta_thetas.append(delta_theta)
        return delta_thetas

    @staticmethod
    def normalise_angle(angle):
        """
        Normalises an angle to the range (-180, 180]
        :param angle: the angle to normalise
        :return: the normalised angle
        """
        while angle <= -180.0:
            angle += 2 * 180.0
        while angle > 180.0:
            angle -= 2 * 180.0
        return angle

    def get_end_state(self, state, index=0):
        """
        Get an end state where 
            state = 0 means puck
            state = 1 means goal

        if there are more than one puck or goal, index can
        be used to get the state of the i'th puck or goal
        """
        if state == 0:
            p = self.scene.pucks[index].shape
        elif state == 1:
            p = self.scene.holes[index].shape
        else:
            return None
        return self.config_from_point(p.centre)

    def visualise_graph(self):
        plt.ylim((-200, 400))
        plt.xlim((-400, 400))
        plotted = []
        for config in self.configs:
            end_pos1 = config.positions[-1]
            for config2 in config.get_connectors().keys():
                end_pos2 = config2.positions[-1]
                if end_pos2 not in plotted:
                    connection = np.array((end_pos1, end_pos2))
                    plt.plot(connection[:, 0], connection[:, 1], color='black',
                             marker='8', linewidth=0.5)
            plotted.append(end_pos1)
        plt.show()

    @staticmethod
    def find_puck_goal_mapping(costs):
        print("Planner: path costs:\n", costs)
        perms = permutations(range(len(costs[0])), len(costs))
        endCosts = []
        permList = []
        permCostList = []
        for i in perms:
            permList.append(i)
            permCosts = []
            for j in range(len(costs)):
                permCosts.append(costs[j][i[j]])

            if None in permCosts:
                endCosts.append(None)
            else:
                endCosts.append(sum(permCosts))

            permCostList.append(permCosts)

        minIndex = None
        minValue = sys.float_info.max
        for x in range(len(endCosts)):
            if endCosts[x] is None:
                continue
            if endCosts[x] < minValue:
                minIndex = x
                minValue = endCosts[x]

        if minIndex is None:
            return None
        return permList[minIndex], permCostList[minIndex]
