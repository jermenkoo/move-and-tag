from mat.robot import Robot
from mat.obstacle import Obstacle
from shapely.geometry import LineString, Polygon, Point
import networkx as nx
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
                    dist = Point(x0, y0).distance(Point(x1, y1))
                    inter = self.intersecting_objs(r1.coord, r2.coord)

                    #initialise point objects
                    a = Point(x0, y0)
                    b = Point(x1, y1)

                    closest = self.closest_int_obst(a, b)

                    if closest:

                        print('%d and %d have obstacles in between and it is %d' % (r1.id, r2.id, closest.id))


                    G.add_edge(r1, r2, {'weight': dist, 'path': []})

        return G





    def intersecting_objs(self, a, b):

        intersecting = []
        a_tupl = a.coords
        b_tupl = b.coords

        for obst in self.obstacles:
            if obst.crosses(a_tupl, b_tupl):
                intersecting.append(obst)

        return intersecting




    def closest_int_obst(self, a, b):

        int_list = self.intersecting_objs(a, b)

        if not int_list:
            return None

        #initial min distance
        closest = int_list[0]
        min_dist = a.distance(closest)

        #loop through all to find the actual closest
        for obst in int_list:
            if a.distance(obst) < min_dist:
                min_dist = a.distance(obst)
                closest = obst


        return closest