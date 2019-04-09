import pyCreate2
import math
import odometry
import pid_controller
import lab8_map
import particle_filter
import pa2_solution
import lab10_map
import RRTree
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
        # self.map = lab8_map.Map("lab8_map.json")
        self.map = lab10_map.Map("configuration_space.png")
        self.RRTree = RRTree.RRTree(1, self.map)
        # self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

        # TODO identify good PID controller gains
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter()

        self.joint_angles = np.zeros(7)

        # the position that the robot returns after localizing
        self.localized_x = 0
        self.localized_y = 0
        self.localized_theta = math.pi / 2

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

        goal_x = waypoints[index][0]
        goal_y = waypoints[index][1]

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
                    if index == len(waypoints):
                        self.create.drive_direct(0,0)
                        break
                    # update the robots new goal
                    goal_x = waypoints[index][0]
                    goal_y = waypoints[index][1]
                    print(goal_x, goal_y)

                # improved version 2: fuse with velocity controller
                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                output_distance = self.pidDistance.update(0, distance, self.time.time())
                self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

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
        x_scale = 3.25 / 300
        y_scale = 3 / 335
        for v in path:
            new_x = (v.x) * x_scale
            new_y = (300 - v.y) * y_scale
            new_vertex = vertex.vertex(new_x, new_y)
            toReturn.insert(0, new_vertex)
        return toReturn

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
        # TAKEN OUT SO NO RED ARROWS
        # self.visualize()
        # self.virtual_create.enable_buttons()
        # self.visualize()

        # hardcoded position for robot to go near arm
        # waypoints = [[.4, 1.6]]
        # self.goto(waypoints)
        # self.go_to_angle(math.pi)

        # TODO this is the code for picking up the cup
        # self.arm.open_gripper()
        # self.arm.go_to(5, -math.pi/3)
        # self.time.sleep(1)
        # self.inverse_kinematics(-0.61, 0.23)
        # self.time.sleep(1)
        # self.inverse_kinematics(-0.67, 0.22)
        # self.time.sleep(1)
        # self.arm.go_to(5, -math.pi/5.5)
        # self.time.sleep(1)
        # self.arm.close_gripper()
        # self.time.sleep(5)
        # self.arm.go_to(1, -math.pi/4)
        # self.time.sleep(1)

        # self.update_odom()
        # self.pf.draw(self.virtual_create)
        # turns_since_wall = 0
        # turn_angle = np.pi / 15.
        # while True:
        #     if (np.array(self.pf.variance()) < np.array([0.02, 0.02, np.pi * 2 / 3])).any():
        #         self.localized_x, self.localized_y, self.localized_theta = self.pf.mean()
        #         break
        #     print('mean', self.pf.mean())
        #     print('variance', self.pf.variance())
        #     self.turn(turn_angle)
        #     dist = self.sense()
        #
        #     if dist > .7:
        #         # We "didn't see a wall."
        #         turns_since_wall += 1
        #     else:
        #         # We "saw a wall."
        #         turns_since_wall = 0
        #
        #     # No wall for three steps, turn back one step and move through the "middle".
        #     if turns_since_wall > 3:
        #         self.turn(-turn_angle)
        #         self.sense()
        #         self.forward(0.5)
        #         self.sense()
        #         turns_since_wall -= 1
        #
        #     self.pf.draw(self.virtual_create)
        #     self.time.sleep(0.01)


        # TODO use localized position to create RRTree
        # TODO need to pass in position to RRTree constructor
        # self.map.draw_line((100, 250), (170, 30), (255, 0, 0))
        for i in range(2000):
            #generate random x
            x_pos = random.randint(0,299)
            y_pos = random.randint(0,299)
            randompoint = vertex.vertex(x_pos,y_pos)
            # print(x_pos,y_pos)
            closest = self.RRTree.find_nearest(randompoint)
            print(closest.x,closest.y)
            actual = self.RRTree.place_point(closest, randompoint)
            if actual is None:
                continue
            if closest.getDistance(actual.x, actual.y) < self.RRTree.delta:
                print("less than delta")
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