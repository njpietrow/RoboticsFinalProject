from pyCreate2 import create2
import lab10_map
import random
import numpy
import vertex
import math
import odometry
import pid_controller


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.map = lab10_map.Map("configuration_space.png")
        self.RRTree = RRTree(1,self.map)

        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pidTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

    def convertPath(self,path):
        toReturn = []
        x_scale = 3.25/300
        y_scale = 3/335
        for v in path:
            new_x = (v.x) * x_scale
            new_y = (300 - v.y) * y_scale
            new_vertex = vertex.vertex(new_x, new_y)
            toReturn.insert(0,new_vertex)
        return toReturn

    def run(self):

        for i in range(2000):
            #generate random x
            x_pos = random.randint(0,299)
            y_pos = random.randint(0,334)
            randompoint = vertex.vertex(x_pos,y_pos)
            closest = self.RRTree.find_nearest(randompoint)
            actual = self.RRTree.place_point(closest, randompoint)
            if actual is None:
                continue
            if closest.getDistance(actual.x, actual.y) < self.RRTree.delta:
                continue
            else:
                print("adding point", i)
                self.RRTree.vertices.append(actual)
                self.RRTree.pairs.append((actual, closest))
                actual.parent = closest

        self.RRTree.draw(self.RRTree.pairs)
        print(len(self.RRTree.vertices))

        solution = self.RRTree.find_nearest(self.RRTree.goal)
        path = []
        while True:
            if solution.parent is not None:
                self.map.draw_line((solution.x, solution.y), (solution.parent.x, solution.parent.y), (0, 0, 255))
                path.append(solution)
                solution = solution.parent
            else:
                break
        self.map.save("pathFP.png")

        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        for i in path:
            print(i.x, i.y)
        path = self.convertPath(path)
        print("-------")
        for i in path:
            print(i.x, i.y)
        waypoints = []
        for i in path:
            waypoints.append([i.x, i.y])

        index = 0

        goal_x = waypoints[index][0]
        goal_y = waypoints[index][1]
        print(goal_x, goal_y)

        # result = numpy.empty((0,5))
        end_time = self.time.time() + 100
        while self.time.time() < end_time:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                # print("[%.6f, %.6f, %.6f]" % (self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                # new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta),
                #            self.odometry.x, self.odometry.y]
                # result = np.vstack([result, new_row])

                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                check = (abs(goal_y) - abs(self.odometry.y) + abs(goal_x) - abs(self.odometry.x))
                if abs(check) < .05:
                    index += 1
                    if index == len(waypoints) -1 :
                        break
                    # update the robots new goal
                    goal_x = waypoints[index][0]
                    goal_y = waypoints[index][1]
                    print(goal_x, goal_y)

                # improved version 2: fuse with velocity controller
                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                output_distance = self.pidDistance.update(0, distance, self.time.time())
                self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))


class RRTree:
    def __init__(self, delta, lab10map):
        self.vertices = []
        # starting point defined below
        self.vertices.append(vertex.vertex(270, 300))
        self.map = lab10map
        self.pairs = []
        self.delta = delta
        self.goal = vertex.vertex(40,120)
        self.waypoints = [[.653,0.27],[.6957,0.33], [.879,0.843], [1.434,0.4611] , [1.943,0.1454], [2.0422, 0.6734],
                          [2.61279, 1.2532],[1.8542, 1.8002], [1.6132, 2.564], [1.87132, 2.4222],[1.912, 2.39822], [0,0]]

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
            if self.map.has_obstacle(int(x_array[i]), int(y_array[i])):
                if i >= 1:
                    return vertex.vertex(x_array[i - 1], y_array[i - 1])
                else:
                    return None
        return vertex2

    def draw(self, array):
        for v in array:
            self.map.draw_line((v[0].x, v[0].y), (v[1].x, v[1].y), (255, 0, 0))

















