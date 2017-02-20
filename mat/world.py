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
                
    def Gsolve(self):
        #magic graph appears
        G=nx.Graph()
        
    def recGoAround(self, start, end):
        path = LineString([start, end])
        for obstacle in self.obstacles:
            #print('path', path)
            if path.crosses(obstacle):
                print('crosses', obstacle)
                #go around obstacle
                #intersections = obstacle.intersection(path)
                #print(intersections)
                print(list(obstacle.exterior.coords))
                #find point we can see A
                pathA = LineString([start, obstacle.exterior.coords[0]])
                edgeA = 0
                while pathA.crosses(obstacle):
                    edgeA += 1
                    pathA = LineString([start, obstacle.exterior.coords[edgeA]])
                    
                print('edge can see A', edgeA)
                
                #find point we can see B
                pathB = LineString([end, obstacle.exterior.coords[0]])
                edgeB = 0
                while pathB.crosses(obstacle):
                    edgeB += 1
                    pathB = LineString([end, obstacle.exterior.coords[edgeB]])
                    
                print('edge can see B', edgeB)
                
                #get edges on shape
                edgeP = (edgeA) % len(obstacle.exterior.coords)
                parts = []
                while edgeP != edgeB:
                    parts.append(obstacle.exterior.coords[edgeP])
                    edgeP = (edgeP + 1) % len(obstacle.exterior.coords)
                    
                parts.append(obstacle.exterior.coords[edgeB])
                print('parts', parts)
                    
                
                return self.recGoAround(start, obstacle.exterior.coords[edgeA]) + parts + self.recGoAround( obstacle.exterior.coords[edgeB], end)
                
                
                #for item in intersections:
                #    print(list(item.coords))
        return []
                
    def Asolve(self):
        #sequentially go from robot 1 to 2, then 3, etc
        currentRobot = 0
        #route = []
        while currentRobot < len(self.robots) - 1:
            start = self.robots[currentRobot].coord
            end = self.robots[currentRobot+1].coord
            self.robots[0].goto(start)
            for position in self.recGoAround(start, end):
                self.robots[0].goto(position)
            self.robots[0].goto(end)
            currentRobot += 1
            #route = route + recGoAround(start, end);

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

                    G.add_edge(r1, r2, {'weight': dist, 'path': []})

        return G
