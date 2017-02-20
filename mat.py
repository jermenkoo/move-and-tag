from mat.world import *

if __name__ == '__main__':
    inFile = open('robots.mat', 'r')
    outFile = open('out.mat', 'w')

    for line in inFile:
        world = World(line)

    inFile.close()
    outFile.close()