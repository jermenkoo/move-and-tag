class Robot:
    def __init__(self, id, coord):
        self.id = id
        self.coord = coord
        self.path = [coord]
        self.alive = True if (id == 0) else False

    def goto(self, coord):
        if coord != self.coord:
            self.path.append(coord)
            self.coord = coord
