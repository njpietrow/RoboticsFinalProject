import lab8_map
import math
import numpy as np
import odometry
import scipy.stats
import lab8_solution as lab8

from particle_filter import ParticleFilter
from pyCreate2 import create2

class Run(lab8.Run):
    def run(self):
        self.create.start()
        self.create.safe()

        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        self.update_odom()

        self.particle_filter.draw(self.virtual_create)

        # Record number of turns since a wall was nearby.
        turns_since_wall = 0
        turn_angle = np.pi / 15.
        while (np.array(self.particle_filter.variance()) > np.array([0.02, 0.02, np.pi * 2/3])).any():
            print('mean', self.particle_filter.mean())
            print('variance', self.particle_filter.variance())
            self.turn(turn_angle)
            dist = self.sense()

            if dist > .7:
                # We "didn't see a wall."
                turns_since_wall += 1
            else:
                # We "saw a wall."
                turns_since_wall = 0

            # No wall for three steps, turn back one step and move through the "middle".
            if turns_since_wall > 3:
                self.turn(-turn_angle)
                self.sense()
                self.forward(0.5)
                self.sense()
                turns_since_wall -= 1

            self.particle_filter.draw(self.virtual_create)
            self.time.sleep(0.01)
