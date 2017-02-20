from mat.robot import Robot
from mat.obstacle import Obstacle
from shapely.geometry import LineString, Polygon, Point
import networkx as nx
import ast


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
                
    def Gsolve(self):
        #magic graph appears
        G=nx.Graph()
        
    def closestEdge(self, start, obstacle):
        #pathA = LineString([start, obstacle.exterior.coords[0]])
        min_path = 100000000000000
        min_edge = 0
        for i in range(len(obstacle.exterior.coords)):
            pathA = LineString([start, obstacle.exterior.coords[i]])
            #print(i, pathA, obstacle, 'fight')
            if not pathA.crosses(obstacle) and not pathA.within(obstacle) and pathA.length < min_path:
            #print(i, pathA, pathA.length, min_path, min_edge)
            #if pathA.length < min_path:
                min_path = pathA.length
                min_edge = i
        return min_edge
                    
        
    def recGoAround(self, start, end, i):
        if i > 500:
            print(self.id, 'depth exceeded')
            self.depth = True
            return []
        if self.depth:
            return []
        path = LineString([start, end])
        #print('path', path)
        for obstacle in self.obstacles:
            if (path.crosses(obstacle) or path.within(obstacle)):
                #print('crosses', obstacle)
                #go around obstacle
                #intersections = obstacle.intersection(path)
                #print(intersections)
                #print(list(obstacle.exterior.coords))
                #find point we can see A
                '''
                pathA = LineString([start, obstacle.exterior.coords[0]])
                edgeA = 0
                while pathA.crosses(obstacle) or pathA.within(obstacle):
                    edgeA += 1
                    pathA = LineString([start, obstacle.exterior.coords[edgeA]])'''
                edgeA = self.closestEdge(start, obstacle)
                    
                #print('edge can see A', edgeA)
                
                #find point we can see B
                ''' pathB = LineString([end, obstacle.exterior.coords[0]])
                edgeB = 0
                while pathB.crosses(obstacle) or pathB.within(obstacle):
                    edgeB += 1
                    pathB = LineString([end, obstacle.exterior.coords[edgeB]])'''
                    
                edgeB = self.closestEdge(end, obstacle)
                #print('edge can see B', edgeB)
                
                #get edges on shape
                edgeP = edgeA
                
                parts = []
                while edgeP != edgeB:
                    parts.append(obstacle.exterior.coords[edgeP])
                    edgeP = (edgeP + 1) % len(obstacle.exterior.coords)
                    
                parts.append(obstacle.exterior.coords[edgeB])
                #print('parts', parts)
                    
               
                return self.recGoAround(start, obstacle.exterior.coords[edgeA], i+1) + parts + self.recGoAround( obstacle.exterior.coords[edgeB], end, i+1)
               
                
                
                #for item in intersections:
                #    print(list(item.coords))
        return []
                
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
            for position in self.recGoAround(start, end, 0):
                #if position not in self.robots[0].path:
                self.robots[0].goto(position)
            self.robots[0].goto(end)
            min_robot.alive = True
            currentRobot = min_robot    
            wakened_robots += 1
            #route = route + recGoAround(start, end);

    def solution(self):
        sol = '{}: '.format(self.id)
        robots_moved = list(filter(lambda r: len(r.path) > 1, self.robots))

        for robot in robots_moved:
            path_str = str(robot.path).replace('[', '').replace(']', '')
            #if len(path_str) < 800000:
            #if not self.depth:
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
