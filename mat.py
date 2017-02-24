#!/usr/bin/env python3
from mat.world import *
import sys
import time
import pickle
import os.path

# from multiprocessing import Pool
from concurrent.futures import ProcessPoolExecutor as Pool

def solveLine(line):
    t0 = time.clock()
    world = World(line)
    #world.visibilityGraph()
    #G = world.graph()
    #pickle.dump(G, open('./graphs/' + world.id + '.graph', 'wb'))
    G = pickle.load(open('./graphs/' + world.id + '.graph', 'rb'))        
    world.EGraphSolve(G)
    print (time.clock() - t0)
    return world.solution()

def fuzzSolveLine(line):
    world = World(line)
    G = pickle.load(open('./graphs/' + world.id + '.graph', 'rb'))        
    world.AGraphSolve(G)
    min_for_world = world.getTotalCost() 
    soln = world.solution()
    print('world', world.id, world.getTotalCost())
    for i in range(1, 1000):
        world = World(line)
        #if world.id == '1' or world.id=='25':
        #world.visibilityGraph()
        #G = world.graph()        
        #pickle.dump(G, open('./graphs/' + world.id + '.graph', 'wb'))
        #if os.path.isfile('./graphs/' + world.id + '.graph'):        
        G = pickle.load(open('./graphs/' + world.id + '.graph', 'rb'))        
        #print(G.edges(data='weight'))
        world.FuzzGraphSolve(G)
        #print (time.clock() - t0)
        if(world.getTotalCost() < min_for_world):
            min_for_world = world.getTotalCost() 
            soln = world.solution()
            print('FOUND world', world.id, world.getTotalCost())
            outFile = open('./fuzz/' + world.id + 'fuzzed' + str(min_for_world) + 'i' + str(i), 'w')
            outFile.write(soln + '\n')
            outFile.close()
        print(i, 'world', world.id, world.getTotalCost(), 'min', min_for_world)
    return soln


if __name__ == '__main__':
    #for i in range(1, 1000):
    inFile = open('robots.mat', 'r')
    outFile = open( 'out.mat', 'w')
    
    outFile.write("lindworm\nf2k19t9k1o1o7i34q7o06jbph6\n")

    result = None
    with Pool(4) as p:
        result = p.map(fuzzSolveLine, inFile.readlines())
        for line in result:
            outFile.write(line + '\n')

    soln = ''

    '''for line in inFile:
        t0 = time.clock()
        min_for_world = float('inf')
        world = World(line)
        if(world.id == '17' or world.id == '1'):
            #baseline
            world = World(line)
            G = pickle.load(open('./graphs/' + world.id + '.graph', 'rb'))        
            world.AGraphSolve(G)
            min_for_world = world.getTotalCost() 
            soln = world.solution()
            print('world', world.id, world.getTotalCost())
            for i in range(1, 1000):
                world = World(line)
                #if world.id == '1' or world.id=='25':
                #world.visibilityGraph()
                #G = world.graph()        
                #pickle.dump(G, open('./graphs/' + world.id + '.graph', 'wb'))
                #if os.path.isfile('./graphs/' + world.id + '.graph'):        
                G = pickle.load(open('./graphs/' + world.id + '.graph', 'rb'))        
                #print(G.edges(data='weight'))
                world.FuzzGraphSolve(G)
                #print (time.clock() - t0)
                if(world.getTotalCost() < min_for_world):
                    min_for_world = world.getTotalCost() 
                    soln = world.solution()
                    print('world', world.id, world.getTotalCost())
                    outFile.write(soln + '\n')
                print(i, 'world', world.id, world.getTotalCost(), 'min', min_for_world)
        print (time.clock() - t0)
        outFile.write(soln + '\n')'''
        
            

    inFile.close()
    outFile.close()
