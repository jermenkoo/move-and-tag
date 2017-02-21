from mat.robot import Robot
from mat.obstacle import Obstacle
from shapely.geometry import LineString, Polygon, Point
import networkx as nx
import ast

from shapely import speedups
speedups.enable()

class World:
    def __init__(self, line):
        self.depth = False
        self.id = None
        self.robots = []
        self.obstacles = []

        # Strip whitespace for convenience
        line = ''.join(line.split())

        self.id = line.split(':')[0]
        print (self.id)
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
                vertexes = list(ast.literal_eval(obstacle_str))
                self.obstacles.append(Obstacle(idx, vertexes))

    def closestVertex(self, start, obstacle):
        paths = map(lambda vertex: (vertex[0], LineString([start, vertex[1]])), enumerate(obstacle.exterior.coords))
        unobstructed_paths = filter(lambda x: not(x[1].crosses(obstacle) or x[1].within(obstacle)), paths)
        closest = min(unobstructed_paths, key=(lambda x: x[1].length))
        return closest[0]

    def obstructions(self, path):
        return list(filter(lambda obs: path.crosses(obs) or path.within(obs), self.obstacles))
    
    #remove hops if previous and next node can see each other
    def clean_hops(self, parts):
        i = 0
        while i < len(parts):
            if 0 == len(self.obstructions(LineString([parts[i], parts[-1]]))):
                parts = parts[0:i+1] + [parts[-1]]
                break
            i+=1
        return parts
    
    def super_clean_hops(self, parts):
        i = 0
        while i < len(parts):
            j = 0
            while j < i:
                if j + 1 < i and len(self.obstructions(LineString([parts[j], parts[i]]))) == 0:
                    parts = parts[0:j+1] + parts[i:]
                    i=j
                j+=1
            i+=1           
        return parts
            

    def border(self, obj, vertexA, vertexB, clockwise=1):
        vertexP = vertexA
        parts = []

        while vertexP != vertexB:            
            #look back            
            parts.append(obj.exterior.coords[vertexP])   
            parts = self.clean_hops(parts)                 
            vertexP = (vertexP + clockwise) % len(obj.exterior.coords)
            

        parts.append(obj.exterior.coords[vertexB])

        return parts

    def shortest_border(self, obj, A, B):
        if A == B:
            return [obj.exterior.coords[A]]

        a_to_b = self.border(obj, A, B)
        b_to_a = self.border(obj, A, B, clockwise=-1)

        if LineString(a_to_b).length < LineString(b_to_a).length:
            return a_to_b
        else:
            return b_to_a

    def recGoAround(self, start, end):
        path = LineString([start, end])
        obstructions = self.obstructions(path)

        if len(obstructions) == 0:
            return []

        else:
            for obs in obstructions:
                vertexA = self.closestVertex(start, obs)
                vertexB = self.closestVertex(end, obs)

                return self.recGoAround(start, obs.exterior.coords[vertexA]) + \
                       self.shortest_border(obs, vertexA, vertexB) + \
                       self.recGoAround(obs.exterior.coords[vertexB], end)

    def fullPath(self, start, end):
        return [start] + self.recGoAround(start, end) + [end]

    def Asolve(self):
        #goto closest robot
        wakened_robots = 1
        currentRobot = self.robots[0]
        #route = []
        while wakened_robots < len(self.robots):
            start = currentRobot.coord
            #first find closest with no obstacle
            min_dist = 100000000000000
            min_robot = None
            for robot in self.robots:
                if robot.coord != currentRobot.coord:
                    if not robot.alive:
                        path = LineString([currentRobot.coord, robot.coord])
                        direct = True
                        for obstacle in self.obstacles:
                            if (path.crosses(obstacle) or path.within(obstacle)):
                                direct = False
                        if direct:                        
                            dist = ((robot.coord[0] - currentRobot.coord[0])**2 + (robot.coord[1] - currentRobot.coord[1])**2) ** 0.5
                            if dist < min_dist:
                                min_dist = dist
                                min_robot = robot  
            
            if min_robot == None:
                min_dist = 100000000000000
                min_robot = None
                for robot in self.robots:
                    if robot.coord != currentRobot.coord:
                        if not robot.alive:
                            dist = ((robot.coord[0] - currentRobot.coord[0])**2 + (robot.coord[1] - currentRobot.coord[1])**2) ** 0.5
                            if dist < min_dist:
                                min_dist = dist
                                min_robot = robot      
                            
            end = min_robot.coord
            self.robots[0].goto(start)
            path = [start] + self.recGoAround(start, end) + [end]
            path = self.super_clean_hops(path)
            
            for position in path:
                #if position not in self.robots[0].path:
                self.robots[0].goto(position)
            self.robots[0].goto(end)
            min_robot.alive = True
            currentRobot = min_robot    
            wakened_robots += 1

    def solution(self):
        sol = '{}: '.format(self.id)
        robots_moved = list(filter(lambda r: len(r.path) > 1, self.robots))

        for robot in robots_moved:
            path_str = str(robot.path).replace('[', '').replace(']', '')
            sol += path_str

            if robot != robots_moved[-1]:
                sol += '; '
        
        return sol

    def graph(self):
        G = nx.DiGraph()

        # Add nodes
        for r in self.robots:
            G.add_node(r)

        for r1 in self.robots:
            for r2 in self.robots:
                if r1 != r2:
                    path = self.fullPath(r1.coord, r2.coord)
                    dist = LineString(path).length

                    G.add_edge(r1, r2, {'weight': dist, 'path': path})

        return G
