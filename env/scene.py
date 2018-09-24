from __future__ import print_function

from conf.settings import BlockSettings, WorkspaceSettings
from env.block import Block
from env.simulator import Simulator

import sys
import os
sys.path.append(os.path.abspath('..//'))


class Scene(object):
    def __init__(self):
        self.blocks = []
        self.nItems = 0
        self.nBlocks = 0
        self.pucks = []
        self.holes = []
        # self.puck = None
        # self.hole = None
        self.moveableBlocks = []
        self.move_block = False
        self.tallFlag = False

    def add_blocks(self, blocks):
        """
        This is useful for adding many blocks at once
        """
        for block in blocks:
            self.add_block(block)

    def find_goals_and_pucks(self):
        for block in self.blocks:
            if block.blockType == BlockSettings.PUCK:
                self.pucks.append(block)
            if block.blockType == BlockSettings.HOLE:
                self.holes.append(block)

    def add_block(self, block):
        if block is None:
            return

        self.nItems += 1

        if block.blockType == BlockSettings.PUCK:
            self.pucks.append(block)
            return

        if block.blockType == BlockSettings.HOLE:
            self.holes.append(block)
            return

        if block.moveable:
            self.moveableBlocks.append(block)
            self.move_block = True

        self.nBlocks += 1
        self.blocks.append(block)

    def to_string(self):
        """
        Convert the scene to a string representation
        """
        sceneString = ""
        sceneString += str(self.nItems) + ',' + str(int(self.tallFlag)) + ','
        for block in self.pucks:
            sceneString += block.to_string()
            sceneString += ','
        for block in self.holes:
            sceneString += block.to_string()
            sceneString += ','
        for block in self.blocks:
            sceneString += block.to_string()
            sceneString += ','
        return sceneString

    def decode(self, sceneString):
        """
        Decode the string generate by to_string and populate the scene instance
        """
        data = sceneString.split(',')
        self.nItems = int(data.pop(0))
        self.tallFlag = float(int(data.pop(0)))
        self.nBlocks = self.nItems
        self.blocks = []
        for i in range(self.nItems):
            block = Block()

            block.decode_from_data(data[0:BlockSettings.nBlockElements])
            if block.blockType == BlockSettings.PUCK:
                self.nBlocks -= 1
                # self.puck = block
                self.pucks.append(block)
            elif block.blockType == BlockSettings.HOLE:
                self.nBlocks -= 1
                # self.hole = block
                self.holes.append(block)
            else:
                self.blocks.append(block)

            if block.moveable:
                if block.tall == self.tallFlag:
                    self.moveableBlocks.insert(0, block)
                else:
                    self.moveableBlocks.append(block)

            data = data[BlockSettings.nBlockElements:]

    def render(self, paths=None):
        sim = Simulator()
        sim.set_axis_limits((-400.0, 400.0), (-200.0, 400.0))
        sim.add_workspace(WorkspaceSettings.WIDTH, WorkspaceSettings.HEIGHT)
        sim.set_scene(self)
        sim.add_arm(180.0, -90.0, 0.0)
        if paths is not None:
            for path in paths:
                sim.add_path(path)
        sim.show_figure()

    def print_scene(self):
        print("Scene: Scene of " + str(self.nBlocks) + " blocks:")
        for i in range(self.nBlocks):
            print("Scene: Block number: ", i)
            self.blocks[i].print_block()
            print("")


if __name__ == "__main__":
    scene = Scene()
    block1 = Block(x=100, y=150, theta=45, blockType=BlockSettings.CUBE,
                   colour=[255, 0, 255], index=0)
    scene.add_block(block1)
    scene.add_block(Block(x=250, y=100, theta=70,
                          blockType=BlockSettings.CYLINDER,
                          colour=[10, 10, 255], index=1))
    # scene.print_scene()
    sceneString = scene.to_string()
    print(sceneString)
    scene2 = Scene()
    scene2.decode(sceneString)
    scene2.print_scene()

    # sim = Simulator()
    # sim.set_axis_limits((-400.0, 400.0), (-200.0, 400.0))
    # sim.add_workspace(WorkspaceSettings.WIDTH, WorkspaceSettings.HEIGHT)
    # sim.set_scene(scene)
    # sim.show_figure()
