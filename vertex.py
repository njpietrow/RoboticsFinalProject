import math


class vertex:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.nb = []

    def addNeighbor(self, neighbor):
        self.nb.append(neighbor)

    def getDistance(self, x, y):
        dx = self.x - x
        dy = self.y - y
        return math.sqrt(pow(dx, 2) + pow(dy, 2))
