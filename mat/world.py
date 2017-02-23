from mat.robot import Robot
from mat.obstacle import Obstacle
from shapely.geometry import LineString, Polygon, Point
import pyvisgraph as vg
import networkx as nx
import ast


from multiprocessing import Pool

import copy

from shapely import speedups
speedups.enable()

class World:
    def __init__(self, line):
        self.depth = False
        self.id = None
        self.robots = []
        self.obstacles = []
        self.vis = None

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

    def visibilityGraph(self):
        polys = map(lambda x: x.vg_poly, self.obstacles)
        g = vg.VisGraph()
        g.build(polys, workers=8)

        # This probably actually makes it slower
        # robots_coords = list(map(lambda r: r.vg_coord, self.robots))
        # g.update(robots_coords)

        self.vis = g

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
            j = 0
            while j < i:
                if j + 1 < i and len(self.obstructions(LineString([parts[j], parts[i]]))) == 0:
                    parts = parts[0:j+1] + parts[i:]
                    i=j
                j+=1
            i+=1

        i = 0
        return parts
            

    def border(self, obj, vertexA, vertexB, clockwise=1):
        vertexP = vertexA
        parts = []

        while vertexP != vertexB:            
            # look back
            parts.append(obj.exterior.coords[vertexP])   
            vertexP = (vertexP + clockwise) % len(obj.exterior.coords)
            

        parts.append(obj.exterior.coords[vertexB])

        return self.clean_hops(parts)

    def shortest_border(self, obj, A, B):
        if A == B:
            return [obj.exterior.coords[A]]

        a_to_b = self.border(obj, A, B)
        b_to_a = self.border(obj, A, B, clockwise=-1)

        if LineString(a_to_b).length < LineString(b_to_a).length:
            return a_to_b
        else:
            return b_to_a

    def recGoAround(self, start, end, rec_depth):
        path = LineString([start, end])
        obstructions = self.obstructions(path)

        if len(obstructions) == 0:
            return []

        else:
            tries = []
            for obs in obstructions:
                vertexA = self.closestVertex(start, obs)
                vertexB = self.closestVertex(end, obs)

                return(self.recGoAround(start, obs.exterior.coords[vertexA], rec_depth+1) + \
                       self.shortest_border(obs, vertexA, vertexB) + \
                       self.recGoAround(obs.exterior.coords[vertexB], end, rec_depth+1))
            if(len(tries[0]) < 2):
                return tries[0]
            min_try = 0
            min_cost = LineString(tries[0]).length
            for i in range(1, len(tries)):
                if LineString(tries[i]).length < min_cost:
                    #print('found cheaper')
                    min_cost = LineString(tries[i]).length
                    min_try = i
            #print(rec_depth)
            return tries[min_try]

    def fullPath(self, start, end):
        return self.clean_hops([start] + self.recGoAround(start, end,0) + [end])

    def solution(self):
        sol = '{}: '.format(self.id)
        robots_moved = list(filter(lambda r: len(r.path) > 1, self.robots))

        for robot in robots_moved:
            path_str = str(robot.path).replace('[', '').replace(']', '')
            sol += path_str

            if robot != robots_moved[-1]:
                sol += '; '
        
        return sol

    def aliveRobots(self):
        return self.aliveRobotsInList(self.robots)
    
    def aliveRobotsInList(self, robots):
        return list(filter(lambda x: x.alive, robots))

    def asleepRobots(self):
        return list(filter(lambda x: not x.alive, self.robots))
    
    def asleepRobotsInList(self, robots):
        return list(filter(lambda x: not x.alive, robots))

    def gotoClosest(self, robot, G):
        return self.gotoClosestInList(robot, G, self.robots)
    
    def gotoClosestInList(self, robot, G, robots):
        node = next(x for x in G.nodes() if robots[x].original_coord == robot.coord)
        out_edges = list(filter(lambda x: not robots[x[0]].alive, list(G[node].items())))
        return min(out_edges, key=lambda x: x[1]['weight'])
    
    def gotoRobot(self, G, roboA, roboB):
        node = next(x for x in G.nodes() if self.robots[x].original_coord == roboA.coord)
        return next(x for x in G[node].items() if x[0] == roboB.id)

    
    def comparePath(self, min_cost, min_paths, my_try):
        return my_try[0] < min_cost
    
        while len(self.asleepRobots()) != 0:
            temp_robots = copy.deepcopy(self.robots)
            self.robots = self.recGSolve(G, temp_robots, 1)[0]
            print('Asleep Robots:', len(self.asleepRobots()))
            '''
            min_cost = float('inf')
            min_paths = 1
            min_path = None
            min_robot = None
            #min_robots = None
            
            if(len(self.asleepRobots()) == 1):
                for robot in self.aliveRobots():
                    if self.gotoClosest(robot, G)[1]['weight'] + robot.time < min_cost:
                        min_path = self.gotoClosest(robot, G)
                        min_cost = min_path[1]['weight'] + robot.time
                        min_robot = robot
                min_robot.time += min_path[1]['weight']
                min_path_robot = self.robots[min_path[0]]
                
                min_path_robot.alive = True
                min_path_robot.time = min_robot.time
                path_taken = min_path[1]['path']
                if path_taken[0] != min_robot.coord:
                    path_taken.reverse()

                for coord in path_taken:
                    min_robot.goto(coord) 
            else:
                for robot in self.aliveRobotsInList(self.robots):
                    for sleep_robot in self.asleepRobotsInList(self.robots):
                        temp_robots = copy.deepcopy(self.robots)
                        temp_robots[sleep_robot.id].alive = True
                        temp_path = self.gotoRobot(G, robot, sleep_robot)
                        temp_cost = temp_path[1]['weight'] + robot.time
                        temp_robots[robot.id].time = temp_cost
                        temp_robots[sleep_robot.id].time = temp_cost
                            
                        temp_try = self.recGSolve(G, temp_robots, 1)                
                        #if self.comparePath(min_cost, min_paths, temp_try[2]):
                        if min_cost > temp_try[1]:
                            min_path = temp_path
                            min_cost = temp_try[1] 
                            min_robot = self.robots[robot.id]
                            min_robots = temp_try[0] 
                #self.robots = min_robots
                
                min_robot.time += min_path[1]['weight']
                min_path_robot = self.robots[min_path[0]]
                
                min_path_robot.alive = True
                min_path_robot.time = min_robot.time
                path_taken = min_path[1]['path']
                if path_taken[0] != min_robot.coord:
                    path_taken.reverse()

                for coord in path_taken:
                    min_robot.goto(coord)
                print('Asleep Robots:', len(self.asleepRobots()))'''
                
    def BOracle(self, G, robots, depth):        
        min_cost = float('inf')
        if depth == 0 or len(self.asleepRobotsInList(robots)) == 1:
            #print(len(self.asleepRobotsInList(robots)), depth)
            for robot in self.aliveRobotsInList(robots):
                if self.gotoClosestInList(robot, G, robots)[1]['weight'] + robot.time < min_cost:
                    min_cost = self.gotoClosestInList(robot, G, robots)[1]['weight'] + robot.time           
        else:
            for robot in self.aliveRobotsInList(robots):
                for sleep_robot in self.asleepRobotsInList(robots):
                    t_robots = copy.deepcopy(robots)                    
                    t_cost = self.gotoRobot(G, robot, sleep_robot)[1]['weight']
                    t_robots[robot.id].time += t_cost
                    t_robots[sleep_robot.id].alive = True
                    t_robots[sleep_robot.id].time = t_robots[robot.id].time
                    t_cost = max(self.BOracle(G, t_robots, depth-1), t_robots[robot.id].time)
                    if(t_cost < min_cost):
                        min_cost = t_cost
        return min_cost
                    
                
    def BGraphSolve(self, G):
        while len(self.asleepRobots()) != 0:
            min_cost = float('inf')
            min_path = None
            min_robot = None
            min_goto_robot = None
            if(len(self.asleepRobots()) == 1):
                self.AGraphSolve(G)
            else:
                for robot in self.aliveRobots():
                    for sleep_robot in self.asleepRobotsInList(self.robots):
                        t_robots = copy.deepcopy(self.robots)                    
                        t_path = self.gotoRobot(G, robot, sleep_robot)
                        t_robots[robot.id].time += t_path[1]['weight']
                        t_robots[sleep_robot.id].alive = True
                        t_robots[sleep_robot.id].time = t_robots[robot.id].time
                        t_cost = max(self.BOracle(G, t_robots, 2), t_robots[robot.id].time)
                        if(t_cost < min_cost):
                            min_cost = t_cost
                            min_path = t_path
                            min_robot = robot
                            min_goto_robot = sleep_robot.id
                            
                min_robot.time += min_path[1]['weight']
                min_path_robot = self.robots[min_goto_robot]
                
                min_path_robot.alive = True
                min_path_robot.time = min_robot.time
                path_taken = min_path[1]['path']
                if path_taken[0] != min_robot.coord:
                    path_taken.reverse()

                for coord in path_taken:
                    min_robot.goto(coord)
        
            print('world', self.id, 'sleeping:', len(self.asleepRobots()))

    def AGraphSolve(self, G):
        while len(self.asleepRobots()) != 0:
            min_cost = float('inf')
            min_path = None
            min_robot = None
            for robot in self.aliveRobots():
                if self.gotoClosest(robot, G)[1]['weight'] + robot.time < min_cost:
                    min_path = self.gotoClosest(robot, G)
                    min_cost = min_path[1]['weight'] + robot.time
                    min_robot = robot
            min_robot.time += min_path[1]['weight']
            min_path_robot = self.robots[min_path[0]]
            
            min_path_robot.alive = True
            min_path_robot.time = min_robot.time
            path_taken = min_path[1]['path']
            if path_taken[0] != min_robot.coord:
                path_taken.reverse()

            for coord in path_taken:
                min_robot.goto(coord)


    def graph(self):
        G = nx.Graph()

        # Add nodes
        for r in self.robots:
            G.add_node(r.id)

        print('# robots: {}'.format(len(self.robots)))

        for r1 in self.robots:
            print('ID', self.id, 'Outer robot: {} out of {}'.format(r1.id, len(self.robots)))
            for r2 in self.robots:
                if r1.id < r2.id:
                    # path = self.fullPath(r1.coord, r2.coord)
                    vg_path = self.vis.shortest_path(vg.Point(*r1.coord), vg.Point(*r2.coord))
                    path = list(map(lambda co: (co.x, co.y), vg_path))

                    dist = LineString(path).length
                    G.add_edge(r1.id, r2.id, {'weight': dist, 'path': path})

        return G
