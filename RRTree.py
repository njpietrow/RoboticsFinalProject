import vertex
import numpy


class RRTree:
    def __init__(self, delta, lab10map):
        self.vertices = []
        # starting point defined below

        self.map = lab10map
        self.pairs = []
        self.delta = delta
        self.goal = vertex.vertex(170,30)

    def find_nearest(self,vertex):
        min_distance = 500
        closest_vertex = None
        for toCheck in self.vertices:
            if toCheck is not vertex:
                distance = toCheck.getDistance(vertex.x, vertex.y)
                if distance < min_distance:
                    closest_vertex = toCheck
                    min_distance = distance
        return closest_vertex

    def place_point(self, vertex1, vertex2):
        x_array = numpy.linspace(vertex1.x, vertex2.x, 800, dtype=numpy.int)
        y_array = numpy.linspace(vertex1.y, vertex2.y, 800, dtype=numpy.int)
        for i in range(800):
            # print(i)
            if self.map.has_obstacle(int(x_array[i]), int(y_array[i])):
                if i >= 1:
                    return vertex.vertex(x_array[i - 1], y_array[i - 1])
                else:
                    return None
        return vertex2

    def draw(self, array):
        for v in array:
            self.map.draw_line((v[0].x, v[0].y), (v[1].x, v[1].y), (255, 0, 0))

