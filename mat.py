#!/usr/bin/env python3
from mat.world import *
import sys
import time

if __name__ == '__main__':
    inFile = open('robots.mat', 'r')
    outFile = open('out.mat', 'w')
    
    outFile.write("lindworm\nf2k19t9k1o1o7i34q7o06jbph6\n")
    
    #hack hack hack
    sys.setrecursionlimit(3000)

    for line in inFile:
        t0 = time.clock()
        world = World(line)
        #if(world.id == '23'):
        #world.Asolve()
        world.AGraphSolve(world.graph())
        print (time.clock() - t0)
        # print(world.solution())
        #outFile.write(world.solution() + '\n')

    inFile.close()
    outFile.close()
