from shapely.geometry import LineString, Polygon
import pyvisgraph as vg

class Obstacle(Polygon):

    #id    : int
    #coord : [(int, int)]
    #vg_poly

    def __init__(self, id, coordinates):
        self.id = id
        super().__init__(coordinates)

        self.vg_poly = []
        for co in coordinates:
            self.vg_poly.append(vg.Point(*co))

    def intersect(self, tup1, tup2):
        path = LineString([tup1, tup2])
        return path.crosses(super())

