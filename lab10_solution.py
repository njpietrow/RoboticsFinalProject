from pyCreate2 import create2
import numpy as np
import math
import odometry
import pid_controller
import lab10_map
import rrt


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.map = lab10_map.Map("lab10.png")
        self.rrt = rrt.RRT(self.map)

    def run(self):

        # find a path
        self.rrt.build((270,300), 3000, 10)
        x_goal = self.rrt.nearest_neighbor((40, 120))
        path = self.rrt.shortest_path(x_goal)

        for v in self.rrt.T:
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        for idx in range(0, len(path)-1):
            self.map.draw_line((path[idx].state[0], path[idx].state[1]), (path[idx+1].state[0], path[idx+1].state[1]), (0,255,0))

        self.map.save("lab10_rrt.png")

        # execute the path (essentially waypoint following from lab 6)
        self.create.start()
        self.create.safe()

        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        self.odometry.x = 2.7
        self.odometry.y = 0.33
        self.odometry.theta = math.pi / 2
        base_speed = 100

        for p in path:
            goal_x = p.state[0] / 100.0
            goal_y = 3.35 - p.state[1] / 100.0
            print(goal_x, goal_y)
            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
                    # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    if distance < 0.05:
                        break
