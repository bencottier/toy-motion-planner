from conf.settings import BlockSettings
from env.geometry import *

import sys
import os

sys.path.append(os.path.abspath('..//'))


class Block(object):
    def __init__(self, x=None, y=None, theta=None,
                 blockType=BlockSettings.UNKNOWN, shape=None,
                 colour = (255,255,255), index=-1, px=0, py=0,
                 moveable=False, tall=False, width=0, height=0):

        colour = (255,255,255)  # change this
        self.tall = int(tall)
        self.x = x
        self.y = y
        self.theta = theta
        self.blockType = blockType
        self.shape = shape
        self.r = int(colour[0])
        self.g = int(colour[1])
        self.b = int(colour[2])
        self.index = index

        self.moveable = int(moveable)

        self.px = int(px)    
        self.py = int(py)

        self.width = width
        self.height = height

        if shape is None:
            self.find_shape()

    def find_shape(self):
        a = BlockSettings.BASIC_LENGTH
        if self.blockType == BlockSettings.CYLINDER:
            self.shape = Circle((self.x, self.y), a/2)

        if self.blockType == BlockSettings.RECT:
            # Put 4 vertices in, anticlockwise. See square below.
            # Use width and height and x, y
            v1 = (self.x-self.width/2, self.y-self.height/2)
            v2 = (self.x-self.width/2, self.y+self.height/2)
            v3 = (self.x+self.width/2, self.y+self.height/2)
            v4 = (self.x+self.width/2, self.y-self.height/2)
            self.shape = Rectangle([v1, v2, v3, v4])

        if self.blockType == BlockSettings.CUBE:
            phi1 = math.radians(45.0 + self.theta)
            phi2 = math.radians(45.0 - self.theta)
            d = a/math.sqrt(2)
            x1 = d*math.cos(phi1)
            y1 = d*math.sin(phi1)
            x2 = d*math.cos(phi2)
            y2 = d*math.sin(phi2)
            v1 = (self.x + x1, self.y + y1)
            v2 = (self.x - x2, self.y + y2)
            v3 = (self.x - x1, self.y - y1)
            v4 = (self.x + x2, self.y - y2)
            self.shape = Rectangle([v1, v2, v3, v4])
        if self.blockType == BlockSettings.PUCK:
            self.shape = Circle((self.x, self.y), BlockSettings.PUCK_DIAMETER/2)
        if self.blockType == BlockSettings.HOLE:
            self.shape = Circle((self.x, self.y), BlockSettings.HOLE_DIAMETER/2)
        if self.blockType == BlockSettings.PRISM:
            phi1 = 180 + self.theta + np.rad2deg(np.arcsin(16/50.0))
            phi3 = phi1 + 2*np.rad2deg(np.arccos(16/50.0))
            phi2 = 90 + self.theta
            r1 = 50
            r2 = 31
            r3 = 50

            x1 = r1*np.cos(np.deg2rad(phi1))
            y1 = r1*np.sin(np.deg2rad(phi1))
            x2 = r2*np.cos(np.deg2rad(phi2))
            y2 = r2*np.sin(np.deg2rad(phi2))
            x3 = r3*np.cos(np.deg2rad(phi3))
            y3 = r3*np.sin(np.deg2rad(phi3))
            
            v1 = (self.x + x1, self.y + y1)
            v2 = (self.x + x2, self.y + y2)
            v3 = (self.x + x3, self.y + y3)

            self.shape = Triangle([v1, v2, v3])

    def to_string(self):
        """ 
        Convert the block to a string representation for sending
        """
        BlockSettings.nBlockElements = 12  # TODO: change in settings as well?
        blockString = "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11}".format(
            str(self.index),
            str(self.blockType),
            str(self.x),
            str(self.y),
            str(self.theta),
            str(self.r),
            str(self.g),
            str(self.b),
            str(self.width),
            str(self.height),
            str(int(self.moveable)),
            str(int(self.tall)))
        return blockString

    def decode(self, blockString):
        """
        Parse the string provided in blockString to populate the block object
        """
        data = blockString.split(",")
        self.decode_from_data(data)

    def decode_from_data(self, data):
        """
        Parse the list of data provided in data to populate the block object
        """
        self.index = int(data[0])
        self.blockType = int(data[1])
        self.x = float(data[2])
        self.y = float(data[3])
        self.theta = float(data[4])
        self.r = int(data[5])
        self.g = int(data[6])
        self.b = int(data[7])
        self.width = float(data[8])
        self.height = float(data[9])
        self.moveable = bool(int(data[10]))
        self.tall = bool(int(data[11]))
        self.find_shape()
        print("Type ", self.blockType)
        print("width, height", self.width, self.height)
        print("x, y", self.x, self.y)
        
    def print_block(self):
        """
        Print the block info for debugging purposes
        """
        # print("----------------------------------")
        print("Block: Index: ", self.index)
        print("Block: Type: ", BlockSettings.TYPE_TO_STR[self.blockType])
        print("Block: Position: ", self.x, self.y)
        print("Block: Theta: ", self.theta)
        print("Block: Colour: ", self.r, self.g, self.b)
        # print("----------------------------------")

    def set_shape(self, shape):
        self.shape = shape


if __name__ == "__main__":
    block = Block(x=100.0, y=150.0, theta=30.0, blockType=BlockSettings.CUBE, 
                  colour=[255, 0, 255], index=0)
    block.print_block()
    blockString = block.to_string()
    print("\n\n")
    newBlock = Block()
    newBlock.decode(blockString)
    newBlock.print_block()
