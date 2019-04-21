import pyCreate2
import math
import odometry
import pid_controller
import particle_filter
import pa2_solution
import lab10_map
import rrt
import vertex
import random

import numpy as np


class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.arm = factory.create_kuka_lbr4p()

        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self.map = lab10_map.Map("configuration_space.png")

        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(300, 0, 20, [0, 0], [-200, 200], is_angle=False)
        # self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        self.rrt = rrt.RRT(self.map)

        self.joint_angles = np.zeros(7)

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def inverse_kinematics(self, x_i, z_i):
        L1 = 0.4 # estimated using V-REP (joint2 - joint4)
        L2 = 0.39 # estimated using V-REP (joint4 - joint6)
        # Corrections for our coordinate system
        z = z_i - 0.3105
        x = -x_i
        # compute inverse kinematics
        r = math.sqrt(x*x + z*z)
        alpha = math.acos((L1*L1 + L2*L2 - r*r) / (2*L1*L2))
        theta2 = math.pi - alpha

        beta = math.acos((r*r + L1*L1 - L2*L2) / (2*L1*r))
        theta1 = math.atan2(x, z) - beta
        print(theta1, theta2)
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            theta2 = math.pi + alpha
            theta1 = math.atan2(x, z) + beta
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            print("Not possible")
            return
        self.time.sleep(.2)
        self.arm.go_to(1, theta1)
        self.arm.go_to(3, theta2)

        print("Go to [{},{}], IK: [{} deg, {} deg]".format(x_i, z_i, math.degrees(theta1), math.degrees(theta2)))

    def sense(self):
        dist = self.sonar.get_distance()
        self.pf.sense(dist)
        return dist

    def turn(self, deltaTheta):
        start = self.odometry.theta
        s = np.sign(deltaTheta)
        self.create.drive_direct(s * 100, s * -100)
        goalStart = deltaTheta + self.odometry.theta
        goalEnd = deltaTheta + self.odometry.theta + s * np.pi / 12
        while True:
            theta = self.odometry.theta
            while s * theta < s * start:
                theta += s * 2 * np.pi
            if s * goalStart <= s * theta <= s * goalEnd:
                break
            self.update_odom()

        self.create.drive_direct(0, 0)
        self.pf.turn(self.odometry.theta - start)

    def update_odom(self):
        state = self.create.update()
        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        self.time.sleep(.01)

    def forward(self, distance):
        start_x = self.odometry.x
        start_y = self.odometry.y
        dist_now = lambda: np.sqrt((start_x - self.odometry.x)**2 +
                                   (start_y - self.odometry.y)**2)
        s = np.sign(distance)
        self.create.drive_direct(s * 100, s * 100)
        while dist_now() < np.abs(distance):
            self.update_odom()
        self.create.drive_direct(0, 0)
        self.pf.forward(dist_now())

    def goto(self, waypoints):
        index = 0
        base_speed = 0

        goal_x = waypoints[index][0]
        goal_y = waypoints[index][1]

        end_time = self.time.time() + 100
        while self.time.time() < end_time:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                output_distance = self.pidDistance.update(0, distance, self.time.time())
                self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
                if distance < 0.009:
                    break

    def go_to_angle(self, goal_theta):
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
                math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)

    def convertPath(self, path):
        toReturn = []
        x_scale = 3 / 300
        y_scale = 3 / 300
        for v in path:
            new_x = (v.x) * x_scale
            new_y = (300 - v.y) * y_scale
            new_vertex = vertex.vertex(new_x, new_y)
            toReturn.insert(0, new_vertex)
        return toReturn

    def goToZero(self, arm, starting):
        for i in range(200):
            self.arm.go_to(arm, 0 + ((200 - i) / 200) * starting)
            self.time.sleep(.01)
        self.arm.go_to(arm, 0)
        self.time.sleep(1)

    def pickUpCup(self):
        self.arm.open_gripper()
        self.arm.go_to(5, -math.pi / 3)
        self.time.sleep(1)
        self.inverse_kinematics(-0.61, 0.23)
        self.time.sleep(1)
        self.inverse_kinematics(-0.67, 0.20)
        self.time.sleep(1)
        self.arm.go_to(5, -math.pi / 5.5)
        self.time.sleep(1)
        self.arm.close_gripper()
        self.time.sleep(5)
        self.goToZero(1, math.pi / 2.7)
        self.goToZero(3, math.pi / 3)
        self.goToZero(5, -math.pi / 6)
        self.time.sleep(1)

    def goFromZero(self, arm, ending, iterations):
        for i in range(iterations):
            self.arm.go_to(arm, ((i + 1) / iterations) * ending)
            self.time.sleep(.01)

    def goToFrom(self, arm, start, end, its):
        for i in range(its):
            self.arm.go_to(arm, start + (end - start) * (i / its))
            self.time.sleep(0.01)

    def placeCup(self, shelf):
        if shelf == 2:
            self.goFromZero(2, math.pi / 2, 100)
            self.goFromZero(3, -math.pi / 2.5, 100)
            self.goToFrom(2, math.pi / 2, math.pi / 6, 100)
            self.goToFrom(5, 0, -math.pi / 9, 10)
            self.time.sleep(5)
            self.arm.open_gripper()
            self.time.sleep(10)
        else:
            self.goFromZero(2, math.pi / 2, 100)
            self.goFromZero(3, -math.pi / 1.78, 100)
            self.goToFrom(2, math.pi / 2, math.pi / 5, 100)
            self.time.sleep(5)
            self.arm.open_gripper()
            self.time.sleep(10)

    def run(self):
        self.create.start()
        self.create.safe()

        self.create.drive_direct(0, 0)

        self.arm.open_gripper()

        self.time.sleep(4)

        self.arm.close_gripper()

        # request sensors
        self.create.start_stream([
            pyCreate2.Sensor.LeftEncoderCounts,
            pyCreate2.Sensor.RightEncoderCounts,
        ])

        self.rrt.build((100, 250), 2000, 10)
        x_goal = self.rrt.nearest_neighbor((165, 75))
        path = self.rrt.shortest_path(x_goal)

        for v in self.rrt.T:
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0, 0, 0))
        for idx in range(0, len(path) - 1):
            self.map.draw_line((path[idx].state[0], path[idx].state[1]),
                               (path[idx + 1].state[0], path[idx + 1].state[1]), (255, 0, 0))

        self.map.save("DirectionMap.png")

        self.odometry.x = 1
        self.odometry.y = .5
        self.odometry.theta = 0
        base_speed = 100

        for p in path:
            goal_x = p.state[0] / 100.0
            goal_y = 3 - p.state[1] / 100.0
            print(goal_x, goal_y)
            while True:
                state = self.create.update()
                if state is not None:

                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(int(base_speed + output_theta), int(base_speed - output_theta))
                    # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
                    if distance < 0.05:
                        break

        self.go_to_angle(0)
        # starting to use particle filter to localize here.
        start_x = path[-1].state[0] / 100
        start_y = 3 - path[-1].state[1] / 100
        self.pf = particle_filter.ParticleFilter(start_x, start_y)
        self.pf.draw(self.virtual_create)


        self.goto([[1.654,2.480]])
        self.go_to_angle(0)
        self.time.sleep(5)

        self.pickUpCup()
        self.placeCup(2)
        self.time.sleep(4)

        # while True:
        #     b = self.virtual_create.get_last_button()
        #     if b == self.virtual_create.Button.MoveForward:
        #         print("Forward pressed!")
        #         self.forward(0.25)
        #     elif b == self.virtual_create.Button.TurnLeft:
        #         print("Turn Left pressed!")
        #         self.turn(np.pi / 6.)
        #     elif b == self.virtual_create.Button.TurnRight:
        #         print("Turn Right pressed!")
        #         self.turn(-np.pi / 6.)
        #     elif b == self.virtual_create.Button.Sense:
        #         self.sense()
        #         print("Sense pressed!")
        #
        #     if b is not None:
        #         self.pf.draw(self.virtual_create)
        #
        #     self.time.sleep(0.01)