#!/usr/bin/env python3
from mat.world import *
import sys
import time
import pickle

# from multiprocessing import Pool
from concurrent.futures import ProcessPoolExecutor as Pool

def solveLine(line):
    t0 = time.clock()
    world = World(line)
    world.visibilityGraph()
    G = world.graph()
    pickle.dump(G, open('./graphs/' + world.id + '.graph', 'wb'))
    world.AGraphSolve(G)
    print (time.clock() - t0)
    return world.solution()


if __name__ == '__main__':
    inFile = open('robots.mat', 'r')
    outFile = open('out.mat', 'w')
    
    outFile.write("lindworm\nf2k19t9k1o1o7i34q7o06jbph6\n")

    result = None
    with Pool(8) as p:
        result = p.map(solveLine, inFile.readlines())
        for line in result:
            outFile.write(line + '\n')


    '''for line in inFile:
        t0 = time.clock()
        world = World(line)
        #world.visibilityGraph()
        #G = world.graph()
        
        #pickle.dump(G, open('./graphs/' + world.id + '.graph', 'wb'))
        
        
        G = pickle.load(open('./graphs/' + world.id + '.graph', 'rb'))
        
        print(G.edges(data='weight'))
        world.AGraphSolve(G)
        print (time.clock() - t0)
        outFile.write(world.solution() + '\n')'''

    inFile.close()
    outFile.close()
