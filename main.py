#!/usr/bin/env python
"""
Run the toy motion planning environment
"""
from __future__ import print_function, division
import env.planner


def demo():
    with open('demo_scene.txt') as f:
        scene_string = f.read()
        planner = env.planner.Planner()
        planner.received_scene_string(scene_string)


if __name__ == '__main__':
    demo()
