import lab8_map
import numpy as np
import odometry
import scipy.stats

from particle_filter import ParticleFilter
from pyCreate2 import create2

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
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create()
        self.odometry = odometry.Odometry()
        self.particle_filter = ParticleFilter()

    def sense(self):
        dist = self.sonar.get_distance()
        self.particle_filter.sense(dist)
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
        self.particle_filter.turn(self.odometry.theta - start)

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
        self.particle_filter.forward(dist_now())

    def run(self):
        self.create.start()
        self.create.safe()

        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.update_odom()

        self.particle_filter.draw(self.virtual_create)
        while True:
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                print("Forward pressed!")
                self.forward(0.25)
            elif b == self.virtual_create.Button.TurnLeft:
                print("Turn Left pressed!")
                self.turn(np.pi / 6.)
            elif b == self.virtual_create.Button.TurnRight:
                print("Turn Right pressed!")
                self.turn(-np.pi / 6.)
            elif b == self.virtual_create.Button.Sense:
                self.sense()
                print("Sense pressed!")

            if b is not None:
                self.particle_filter.draw(self.virtual_create)

            self.time.sleep(0.01)
