from mat.robot import Robot
from mat.obstacle import Obstacle
from shapely.geometry import LineString, Polygon

import ast


class World:
    id = None
    robots = []
    obstacles = []

    def __init__(self, line):
        # Strip whitespace for convenience
        line = ''.join(line.split())

        self.id = line.split(':')[0]
        robots_str = line.split(':')[1].split('#')[0]
        obstacles_str = None

        try:
            obstacles_str = line.split(':')[1].split('#')[1]
        except:
            pass    # No obstacles

        # Read robots
        if robots_str:
            robots_list = list(ast.literal_eval(robots_str))
            for idx, coord in enumerate(robots_list):
                self.robots.append(Robot(idx, coord))

        # Read obstacles
        if obstacles_str:
            for idx, obstacle_str in enumerate(obstacles_str.split(';')):
                vertexes = list(ast.literal_eval(obstacles_str))
                self.obstacles.append(Obstacle(idx, vertexes))
                
    def Asolve(self):
        #sequentially go from robot 1 to 2, then 3, etc
        currentRobot = 0
        #route = []
        while currentRobot < len(robots):
            start = robots[currentRobot].myCoord
            end = robots[currentRobot+1].myCoord
            for position in recGoAround(start, end):
                robots[currentRobot].goto(position)
            currentRobot += 1
            #route = route + recGoAround(start, end);


    def recGoAround(self, start, end):
        path = LineString[start, end]
        for obstacle in obstacles:
            if path.crosses(obstacle):
                #go around obstacle
                intersections = obstacle.intersection(path)
        return [start, end]
                    
            

