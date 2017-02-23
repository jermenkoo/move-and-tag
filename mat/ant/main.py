import re
from os import path
import glob
from .dmst import AB_DCMST

if __name__ == '__main__':

    for filePath in glob.glob('Data/shrd*.txt'):
        print (filePath)
        nbVertices = getNbVertices(filePath)
        edges = getEdges(filePath, nbVertices)
        #print 'Vertices :', nbVertices, 'Edges :', len(edges)
        
        antBasedSolver = AB_DCMST(edges)
        for constraint in range(3, 4):
            tree = antBasedSolver.getSolution(constraint)
            print ('{}\t{}'.format(constraint, getTreeCost(tree)))
        #print