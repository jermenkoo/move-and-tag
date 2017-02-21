#!/usr/bin/env python3
from mat.world import *

if __name__ == '__main__':
    inFile = open('robots.mat', 'r')
    outFile = open('out.mat', 'w')
    count = 0

    for line in inFile:

        world = World(line)
        world.graph()

        print(world.solution())
        count += 1

    inFile.close()
    outFile.close()