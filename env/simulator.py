from __future__ import print_function

from env.robot import Robot
from conf.settings import *

import matplotlib
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np

import sys
import os
sys.path.append(os.path.abspath('..//'))

COLORS = 10*['blue', 'green', 'red', 'yellow', 'magenta', 'cyan']


class Simulator(object):

    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        # self.fig, self.ax = plt.subplots()
        self.robot = Robot()
        self.obstacles = []
        self.goals = []
        self.pucks = []

        self.endPoints = None
        self.elbowPoints = None
        self.pathPoints = None
        self.paths = []

        self.pathColour = 'black'
        self.goalColour = 'cyan'
        self.puckColour = 'gray'
        self.obstacleColour = 'red'
        self.workspaceFeatures = []
        self.workspaceColour = 'green'
        self.armColour = 'black'

        # plt.ion()

    def add_goal(self, xy, radius):
        circle = Circle(xy, radius)
        self.goals.append(circle)

    def add_puck(self, xy, radius):
        circle = Circle(xy, radius)
        self.pucks.append(circle)

    def add_obstacle_polygon(self, points):
        self.obstacles.append(Polygon(points))

    def add_obstacle_circle(self, xy, radius):
        circle = Circle(xy, radius)
        self.obstacles.append(circle)

    def set_axis_limits(self, x_limits, y_limits):
        plt.ylim(y_limits)
        plt.xlim(x_limits)

    def add_workspace(self, width, height):
        xleft = 0.0 - width/2
        xright = 0.0 + width/2
        ylower = 0.0
        yupper = height
        points = np.array([[xleft, ylower], [xright, ylower], [xright, yupper],
                           [xleft, yupper]])
        self.workspaceFeatures.append(Polygon(points))

    def add_arm(self, theta1, theta2, theta3=0):
        self.elbowPoints = self.robot.get_elbow_position(theta1)
        self.endPoints = self.robot.forward_kinematics(theta1, theta2, theta3)[0:2]
        
    def add_path(self, path):
        self.paths.append(path)

    def set_scene(self, scene=None):
        for puck in scene.pucks:
            self.add_puck(puck.shape.centre, puck.shape.radius)
        for hole in scene.holes:
            self.add_goal(hole.shape.centre, hole.shape.radius)

        for block in scene.blocks:
            shape = block.shape
            if shape is None:
                continue
            if shape.type == "circle":
                self.add_obstacle_circle(shape.centre, shape.radius)
            else:
                vertices = np.array([list(v) for v in shape.vertices])
                self.add_obstacle_polygon(vertices)

    def show_figure(self):
        # plt.figure(1)
        # plt.clf()
        # plt.close()

        # self.set_axis_limits((-400.0, 400.0), (-200.0, 400.0))
        # self.add_workspace(WorkspaceSettings.WIDTH, WorkspaceSettings.HEIGHT)
        # self.set_scene()

        # self.add_arm(180.0, -90.0, 0.0)

        # if len(self.paths) == 0:
        #      plt.close()
        p = PatchCollection(self.workspaceFeatures, alpha=0.1,
                            color=self.workspaceColour, edgecolor='black')
        self.ax.add_collection(p)

        p = PatchCollection(self.obstacles, alpha=0.9,
                            color=self.obstacleColour, edgecolor='black')
        self.ax.add_collection(p)

        p = PatchCollection(self.goals, alpha=0.9, color=self.goalColour,
                            edgecolor='black')
        self.ax.add_collection(p)

        p = PatchCollection(self.pucks, alpha=0.9, color=self.puckColour,
                            edgecolor='black')
        self.ax.add_collection(p)

        if len(self.paths) > 0:
            i = 0
            for path in self.paths:
                plt.plot(path[:, 0], path[:, 1],
                         color=COLORS[i], solid_capstyle='round', alpha=0.2,
                         linewidth=BlockSettings.PUCK_DIAMETER/2)
                plt.plot(path[:, 0], path[:, 1],
                         color=self.armColour, linestyle='--', linewidth=2)
                i += 1

        if self.elbowPoints is not None:
            plt.plot([0, self.elbowPoints[0], self.endPoints[0]],
                     [-150, self.elbowPoints[1], self.endPoints[1]],
                     color=self.armColour, linestyle='-', linewidth=5)
        # plt.pause(0.00001)
        # self.fig.canvas.draw()
        # plt.draw()
        # plt.pause(1)
        
        plt.show()


if __name__ == '__main__':
    sim = Simulator()
    sim.set_axis_limits((-400, 400), (-200, 400))
    sim.add_workspace(WorkspaceSettings.WIDTH, WorkspaceSettings.HEIGHT)

    # Add an arm with (theta1, theta2, theta3)
    sim.add_arm(10.38, 90, 0)
    
    sim.add_goal([150, 150], 25)

    # sim.add_path(np.array([[0,0],[100,100],[150,200]]))
    
    sim.add_puck([250, 150], BlockSettings.PUCK_DIAMETER/2)
    
    # Add a triangle
    pts = np.array([[0, 50], [100, 100], [0, 200]])
    sim.add_obstacle_polygon(pts)

    # Add a circle
    sim.add_obstacle_circle([-200, 85], 25)

    # Add a quad
    pts = np.array([[-50, 50], [-50, 100], [-100, 100], [-100, 50]])
    sim.add_obstacle_polygon(pts)

    sim.show_figure()
