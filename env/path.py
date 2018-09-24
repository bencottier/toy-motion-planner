from __future__ import print_function

import sys
import os
sys.path.append(os.path.abspath('..//'))


class Path(object):

    def __init__(self):
        self.path = []
        self.angles = []
        self.deltaTheta = 0
        self.cost = None
        self.movingPath = False

    def to_string(self):
        """
        Encode the path as a string representation for sending
        """
        # print("Angle is: ", self.deltaTheta)
        pathString = str(int(self.movingPath)) + "," + str(int(self.deltaTheta)) + "," 
        for i in self.path:
            pathString += "{0},{1},".format(i.angles[0], i.angles[1])

        return pathString

    def decode(self, pathString):
        """
        Populate class based on the data in pathString
        """
        data = pathString.split(',')
        self.movingPath = bool(int(data.pop(0)))
        self.deltaTheta = int(data.pop(0))
        for i in range((len(data)-1)/2):
            self.angles.append([float(data[2*i]), float(data[2*i + 1])])

    def print_path(self):
        for i in self.path:
            print(i.angles)

    def set_path(self, path):
        self.path = path
        # print("Path: " + self.to_string())


if __name__ == "__main__":
    path = Path()
    pathString = "1.0,2,3.34,4,5,6,7,8,9,0,"
    path.decode(pathString)
    print(path.angles)

