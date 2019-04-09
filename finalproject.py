import pyCreate2
import math
import odometry
import pid_controller
import lab8_map
import particle_filter

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
        self.map = lab8_map.Map("lab8_map.json")
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

        # TODO identify good PID controller gains
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(self.map, 1000, 0.06, 0.15, 0.2)

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

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def goto(self):
        waypoints = [[1.5,2.75]]
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
                    if index == 1:
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


        self.goto()


        while True:
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.forward()
                self.visualize()
            elif b == self.virtual_create.Button.TurnLeft:
                self.go_to_angle(self.odometry.theta + math.pi / 2)
                self.visualize()
            elif b == self.virtual_create.Button.TurnRight:
                self.go_to_angle(self.odometry.theta - math.pi / 2)
                self.visualize()
            elif b == self.virtual_create.Button.Sense:
                distance = self.sonar.get_distance()
                print(distance)
                self.pf.measure(distance, 0)
                self.visualize()

            self.time.sleep(0.01)