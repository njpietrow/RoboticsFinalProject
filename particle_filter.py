import numpy as np
import scipy.stats


class ParticleFilter:
    def __init__(self, num_particles=3000, sensor_sigma=0.1, the_map=None,
                 cartesian_sigma=0.05, theta_sigma=np.pi/48.):
        self.num_particles = num_particles
        self.drawn_particles = 500
        self.sensor_sigma = sensor_sigma
        if the_map is None:
            import lab8_map
            the_map = lab8_map.Map("lab8_map.json")
        self.map = the_map
        self.cartesian_sigma = cartesian_sigma
        self.theta_sigma = theta_sigma

        self.particles = (np.random.uniform(
            low=self.num_particles * [0, 0, 0],
            high=self.num_particles * [3, 3, 2 * np.pi])
            .reshape(self.num_particles, 3))

    def mean(self):
        '''Compute the mean of the particle positions.

        This is useful as an estimate of where the robot is.
        '''
        x, y, theta = self.particles.mean(0)
        return x, y, theta

    def variance(self):
        '''Compute the variance of the particle positions.

        This is useful for estimating how certain the location estimate is.
        '''
        x, y, theta = self.particles.var(0)
        return x, y, theta

    def draw(self, virtual_create):
        x, y, theta = self.particles.mean(0)
        virtual_create.set_pose((x, y, 0.1), theta)

        # The format is x,y,z,theta,x,y,z,theta,...
        data = np.ones((self.drawn_particles, 4))
        data[:, 0] = self.particles[:self.drawn_particles, 0]
        data[:, 1] = self.particles[:self.drawn_particles, 1]
        data[:, 2] = 0.1
        data[:, 3] = self.particles[:self.drawn_particles, 2]
        virtual_create.set_point_cloud(data.flatten())

    def sense(self, measured_distance):
        '''Update particles using a sonar measurement.
        
        Assumes the sonar is facing straight forward.'''
        expected_distances = np.array([
            self.map.closest_distance((self.particles[i, 0], self.particles[i, 1]),
                                      self.particles[i, 2]) or 3.
            for i in range(self.num_particles)])
        # Baye's rule (in extremely simplified form).
        p_s_o = scipy.stats.norm(measured_distance, self.sensor_sigma).pdf(expected_distances)
        # Normalize to get a probability again.
        p_s_o /= np.sum(p_s_o)
        # Sample.
        idx = np.random.choice(self.num_particles, self.num_particles, p=p_s_o)
        # If the particles were *objects* (as opposed to elements in an numpy
        # array), we would need to deepcopy here.
        self.particles = self.particles[idx]

    def turn(self, deltaTheta):
        '''Record that the robot rotated by deltaTheta.'''
        self.particles[:, 2] += deltaTheta
        self.particles[:, 2] += np.random.normal(0, self.theta_sigma, self.num_particles)
        self.particles[:, 2] = np.fmod(self.particles[:, 2], 2 * np.pi)

        # Add x,y noise, even though we only turned.
        self.particles[:, 0] += np.random.normal(0, self.cartesian_sigma, self.num_particles)
        self.particles[:, 1] += np.random.normal(0, self.cartesian_sigma, self.num_particles)

    def forward(self, distance):
        '''Record that the robot moved (forward) by distance.'''
        # Update x, y using *particle's* theta.
        self.particles[:, 0] += distance * np.cos(self.particles[:, 2])
        self.particles[:, 1] += distance * np.sin(self.particles[:, 2])

        # Add noise.
        self.particles[:, 0] += np.random.normal(0, self.cartesian_sigma, self.num_particles)
        self.particles[:, 1] += np.random.normal(0, self.cartesian_sigma, self.num_particles)
        self.particles[:, 2] += np.random.normal(0, self.theta_sigma, self.num_particles)
        self.particles[:, 2] = np.fmod(self.particles[:, 2], 2 * np.pi)
