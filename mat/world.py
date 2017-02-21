from mat.robot import Robot
from mat.obstacle import Obstacle
from shapely.geometry import LineString, Polygon, Point
import networkx as nx
import matplotlib.pyplot as plt
import ast


class World:
    def __init__(self, line):
        self.id = None
        self.robots = []
        self.obstacles = []

        # Strip whitespace for convenience
        line = ''.join(line.split())

        self.id = line.split(':')[0]
        robots_str = line.split(':')[1].split('#')[0]
        obstacles_str = None

        try:
            obstacles_str = line.split(':')[1].split('#')[1]
        except:
            pass  # No obstacles

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

    def Gsolve(self):
        # magic graph appears
        G = nx.Graph()

    def Asolve(self):
        # sequentially go from robot 1 to 2, then 3, etc
        currentRobot = 0
        # route = []
        while currentRobot < len(robots):
            start = robots[currentRobot].myCoord
            end = robots[currentRobot + 1].myCoord
            for position in recGoAround(start, end):
                robots[currentRobot].goto(position)
            currentRobot += 1
            # route = route + recGoAround(start, end);

    def recGoAround(self, start, end):
        path = LineString[start, end]
        for obstacle in obstacles:
            if path.crosses(obstacle):
                # go around obstacle
                intersections = obstacle.intersection(path)
        return [start, end]

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
                    x0, y0 = r1.coord
                    x1, y1 = r2.coord

                    closest = self.closest_int_obst(r1.coord, r2.coord)

                    if not closest:
                        dist = Point(x0, y0).distance(Point(x1, y1))
                        G.add_edge(r1, r2, {'weight': dist, 'path': []})
                    else:
                        tempG = nx.Graph()
                        self.construct_subGraph(r1.coord, r2.coord, tempG)
                        self.construct_subGraph(r2.coord, r1.coord, tempG)
                        path = nx.shortest_path(tempG, r1.coord, r2.coord, weight='weight')
        return G


    def construct_subGraph(self, a, b, final):
        closest = self.closest_int_obst(a, b)
        #add both points
        if (a not in final.nodes()):
            final.add_node(a)

        #if unobstructed, add direct edge
        if not closest:
            if (b not in final.nodes()):
                final.add_node(b)
            dist = Point(a).distance(Point(b))
            final.add_edge(a,b,{'weight' : dist})

        else:
            a_no_self_obs = []
            b_no_self_obs = []

            obstacle_nodes = list(closest.exterior.coords)
            for cl in obstacle_nodes:
                if cl not in final.nodes():
                    final.add_node(cl)
                if not closest.crosses(cl, a):
                    a_no_self_obs.append(cl)
                if not closest.crosses(cl, b):
                    b_no_self_obs.append(cl)

            for i in range(len(obstacle_nodes)):
                n1 = obstacle_nodes[i]
                n2 = obstacle_nodes[(i+1)%len(obstacle_nodes)]
                dist = Point(n1).distance(Point(n2))
                final.add_edge(n1, n2, {'weight' : dist})

            for a_no in a_no_self_obs:
                dist = Point(a).distance(Point(a_no))
                final.add_edge(a, a_no, {'weight' : dist})

            min_node = b_no_self_obs[0]
            min_dist = dist = Point(min_node).distance(Point(b))
            for b_no in b_no_self_obs:
                dist = Point(b_no).distance(Point(b))
                if dist < min_dist:
                    min_node = b_no
                    min_dist = dist


            self.construct_subGraph(min_node, b, final)



    def intersecting_objs(self, a, b):

        intersecting = []

        for obst in self.obstacles:
            if obst.crosses(a, b):
                intersecting.append(obst)

        return intersecting




    def closest_int_obst(self, a, b):

        int_list = self.intersecting_objs(a, b)

        if not int_list:
            return None

        #initial min distance
        closest = int_list[0]
        min_dist = Point(a).distance(closest)

        #loop through all to find the actual closest
        for obst in int_list:
            if Point(a).distance(obst) < min_dist:
                min_dist = Point(a).distance(obst)
                closest = obst


        return closest