#!/usr/bin/env python3
from mat.world import *

if __name__ == '__main__':
    inFile = open('robots.mat', 'r')
    outFile = open('out.mat', 'w')
    
    outFile.write("lindworm\nf2k19t9k1o1o7i34q7o06jbph6\n")

    for line in inFile:
        world = World(line)
        world.Asolve()
        outFile.write(world.solution() + '\n')

    inFile.close()
    outFile.close()
