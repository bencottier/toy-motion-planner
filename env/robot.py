from __future__ import print_function

from conf.settings import RobotSettings

import numpy as np
import sys
import os

sys.path.append(os.path.abspath('..//'))

DOWN_HEIGHT = 3  # 5
TRANSIT_SPEED = 50
INIT_MOVE_SPEED = 50
MOVE_BLOCK_HEIGHT = 15
MOVE_BLOCK_SPEED = 15


class Robot(object):
    def __init__(self):
        self.link1Length = float(RobotSettings.LINK_1_LENGTH)
        self.link2Length = float(RobotSettings.LINK_2_LENGTH)
        self.baseXOffset = RobotSettings.BASE_X_OFFSET
        self.baseYOffset = RobotSettings.BASE_Y_OFFSET

    def get_elbow_position(self, theta1):
        """
        Get the x,y position of the elbow. Useful for the simulator drawing.
        """
        t1 = np.deg2rad(theta1)
        x = self.link1Length * np.cos(t1)
        y = self.link1Length * np.sin(t1) - self.baseYOffset
        return [x, y]

    def forward_kinematics(self, theta1, theta2, theta3, all_joints=False):
        """
        Convert joint space to work space.
        Input in degrees
        """
        t1 = np.deg2rad(theta1)
        t2 = np.deg2rad(theta2)
        t3 = np.deg2rad(theta3)

        x1 = self.baseXOffset
        y1 = -self.baseYOffset

        x2 = x1 + self.link1Length * np.cos(t1)
        y2 = y1 + self.link1Length * np.sin(t1)
        x3 = x2 + self.link2Length*np.cos(t1 + t2)
        y3 = y2 + self.link2Length*np.sin(t1 + t2)

        theta = theta1 + theta2 + theta3

        if all_joints:
            return [(x1, y1), (x2, y2), (x3, y3), theta]

        return [x3, y3, theta]

    def inverse_kinematics(self, x, y, z=0, theta=0):
        """
        Drive the end effector to the world space coodinate x,y,z rotation theta.
        Theta in degrees.
        """
        x = float(x) - self.baseXOffset
        y = float(y) + self.baseYOffset
        # print("Robot: {0}, {1}".format(self.link1Length, self.link2Length))
        if x == 0:
            x = 0.0000001  # Don't got time for divide by 0 errors

        theta2 = np.rad2deg(np.arccos(
            (x**2 + y**2 - self.link1Length**2 - self.link2Length**2) /
            (2*self.link1Length*self.link2Length)))

        d = np.sqrt(x**2 + y**2)  # distance from base to point
        phi = np.rad2deg(np.arcsin(self.link2Length*np.sin(
            np.deg2rad(theta2))/d))
        temp = np.rad2deg(np.arctan(abs(y/x)))
        if x < 0:
            temp = 180 - temp
        theta1 = temp - phi
        theta3 = theta - theta1 - theta2

        if np.isnan(theta1) or np.isnan(theta2) or np.isnan(theta3):
            print("Robot: Invalid position received in IK: (x,y,z,theta): ",
                  x, y, z, theta)
            exit()
        # These are the joint angles, move to them
        return theta1, theta2, theta3
