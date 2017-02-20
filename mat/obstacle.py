from shapely.geometry import LineString, Polygon

class Obstacle(Polygon):

    #id    : int
    #coord : [(int, int)]

    def __init__(self, id, coordinates):
        self.id = id
        super().__init__(coordinates)

    def intersect(self, tup1, tup2):
        path = LineString([tup1, tup2])
        path.intersects(super())
