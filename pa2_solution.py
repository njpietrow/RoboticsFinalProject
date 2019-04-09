"""
Sample Solution for PA2
Use "run.py [--sim] pa2_solution" to execute
"""
import math
import numpy as np


class Run:
    def __init__(self, arm, timeHelper):
    # def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactorySimulation)
        """
        # self.arm = factory.create_kuka_lbr4p()
        # self.time = factory.create_time_helper()
        self.arm = arm
        self.time = timeHelper




    def forward_kinematics(self, theta1, theta2):
        self.arm.go_to(1, theta1)
        self.arm.go_to(3, theta2)
        L1 = 0.4 # estimated using V-REP (joint2 - joint4)
        L2 = 0.39 # estimated using V-REP (joint4 - joint6)
        z = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2) + 0.3105
        x = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
        print("Go to {},{} deg, FK: [{},{},{}]".format(math.degrees(theta1), math.degrees(theta2), -x, 0, z))

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

        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            theta2 = math.pi + alpha
            theta1 = math.atan2(x, z) + beta
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            print("Not possible")
            return

        self.arm.go_to(1, theta1)
        self.arm.go_to(3, theta2)
        print("Go to [{},{}], IK: [{} deg, {} deg]".format(x_i, z_i, math.degrees(theta1), math.degrees(theta2)))

    def run(self):
        # Arrange the endeffector such that it faces the wall
        # self.arm.go_to(0, math.radians(-90))
        # self.time.sleep(5)
        # self.arm.go_to(0, math.radians(90))
        # self.time.sleep(5)
        self.arm.go_to(4, math.radians(-90))
        self.arm.go_to(5, math.radians(90))
        self.time.sleep(100)

        # print("Using joint 1 (range -90 to 90 degrees)")
        # print("Using joint 3 (range -90 to 90 degrees)")

        # # test forward kinematics
        # self.forward_kinematics(math.radians(60), math.radians(-70))
        # self.time.sleep(1.0)
        # self.forward_kinematics(math.radians(45), math.radians(-45))
        # self.time.sleep(1.0)
        # self.forward_kinematics(math.radians(90), math.radians(90))
        # self.time.sleep(1.0)
        # self.forward_kinematics(math.radians(-70), math.radians(45))
        # self.time.sleep(1.0)

        # # test inverse kinematics
        # self.arm.set_color(0.0, 0.0, 1.0)
        # self.inverse_kinematics(-0.3, 1.0)
        # self.time.sleep(1.0)
        # self.arm.enable_painting()
        # self.inverse_kinematics(0.3, 1.0)
        # self.time.sleep(1.0)
        # self.inverse_kinematics(0.3, 0.9)
        # self.time.sleep(1.0)
        # self.inverse_kinematics(-0.3, 0.9)
        # self.time.sleep(1.0)
        # self.inverse_kinematics(-0.3, 1.0)
        # self.time.sleep(1.0)

        #better square
        self.inverse_kinematics(-0.3, 1.0)
        self.time.sleep(1.0)
        self.arm.set_color(0.0, 0.0, 1.0)
        self.arm.enable_painting()
        for x in np.arange(-0.3, 0.3, 0.01):
            self.inverse_kinematics(x, 1.0)
            self.time.sleep(0.05)
        self.arm.set_color(0.0, 1.0, 0.0)
        for z in np.arange(1.0, 0.9, -0.01):
            self.inverse_kinematics(0.3, z)
            self.time.sleep(0.05)
        self.arm.set_color(1.0, 0.0, 0.0)
        for x in np.arange(0.3, -0.3, -0.01):
            self.inverse_kinematics(x, 0.9)
            self.time.sleep(0.05)
        self.arm.set_color(0.0, 1.0, 1.0)
        for z in np.arange(0.9, 1.0, 0.01):
            self.inverse_kinematics(-0.3, z)
            self.time.sleep(0.05)

        self.time.sleep(10)