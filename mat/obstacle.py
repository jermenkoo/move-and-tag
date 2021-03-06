from shapely.geometry import LineString, Polygon


class Obstacle(Polygon):
    # id    : int
    # coord : [(int, int)]

    def __init__(self, id, coordinates):
        self.id = id
        super().__init__(coordinates)

    def crosses(self, tup1, tup2):
        path = LineString([tup1, tup2])
        return path.crosses(super())
