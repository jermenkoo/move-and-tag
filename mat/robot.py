import pyvisgraph as vg

class Robot:
    def __init__(self, id, coord):
        self.id = id

        self.original_coord = coord
        self.coord = coord
        self.vg_coord = vg.Point(*coord)

        self.path = [coord]
        self.alive = True if (id == 0) else False

    def goto(self, coord):
        if coord != self.coord:
            self.path.append(coord)
            self.coord = coord
