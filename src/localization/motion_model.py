"""
Notes:
- Pre-compute cosines and sines
- How do we get "deterministic" and "num_particles" parameters into this file (for integration)
"""

import rospy
import numpy as np

class MotionModel:

    def __init__(self):

        self.deterministic = rospy.get_param("~deterministic")

        # Pre-compute cosines, sines, arcsines, and arccosines:
            # Every time we run vec2pose and pose2vec it uses cosines, sines, arccosines, and arcsines
            # So we should make a map of theta --> costheta, and costheta --> theta, etc.
            # Currently not implemented -- just using np.sin, np.cos, np.arccos, np.arcsin in helper functions
            # Is this worth our time?

        # Pre-computing gaussian noise
        # Sigma values are the parameters we tune!
        xmu, xsig = 1, 3
        ymu, ysig = 1, 1
        tmu, tsig = 1, 3
        self.points = 1000
        self.noise = np.random.normal([xmu, ymu, tmu], [xsig, ysig, tsig], size = (self.points, 3))

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        # if self.deterministic:
        #
        #     for i in range(len(particles)):
        #
        #         cos = np.cos(particles[i, 2])
        #         sin = np.sin(particles[i, 2])
        #
        #         x = cos * odometry[0] - sin * odometry[1] + particles[i, 0]
        #         y = sin * odometry[0] + cos * odometry[1] + particles[i, 1]
        #         t = odometry[2] + particles[i, 2]
        #
        #         particles[i] = [x, y, t]
        #
        # else:
        #
        #     for i in range(len(particles)):
        #
        #         # Pick a random noise from our pre-computed gaussian samples
        #         noise = self.noise[np.random.randint(1, self.points)]
        #         noisy_odom = [odometry[0]*noise[0], odometry[1]*noise[1], odometry[2]*noise[2]]
        #
        #         cos = np.cos(particles[i, 2])
        #         sin = np.sin(particles[i, 2])
        #
        #         x = cos * noisy_odom[0] - sin * noisy_odom[1] + particles[i, 0]
        #         y = sin * noisy_odom[0] + cos * noisy_odom[1] + particles[i, 1]
        #         t = noisy_odom[2] + particles[i, 2]
        #
        #         particles[i] = [x, y, t]
        #
        # return particles

        odoms = np.tile(np.array(odometry), (len(particles), 1))
        if not self.deterministic:
            # odoms = np.tile(np.array(odometry), (1, len(self.particles)))
        # else:
            noises = []
            for i in range(len(particles)):
                noises.append(self.noise[np.random.randint(1, self.points)])
            odoms = np.multiply(odoms, noises)

        thetas = np.array([particles[:,2]])
        cosines = np.cos(thetas)
        sines = np.sin(thetas)
        zeros = np.zeros(thetas.shape)
        ones = np.ones(thetas.shape)
        Ms_unshaped = np.concatenate((cosines, -1*sines, zeros, sines, cosines, zeros, zeros, zeros, ones), axis=1)
        Ms = Ms_unshaped.reshape([len(particles), 3, 3])
        # print(Ms[20])
        changes = np.matmul(Ms, odoms.reshape([len(particles), 3, 1]))
        particles = np.add(changes.reshape([len(particles), 3]), particles)

        return particles


        # x_array = np.array([point[:, 0]]).T
        # y_array = np.array([point[:,1]]).T
        # thetas = np.array([point[:,-1]])
        # zero = np.zeros(x_array.shape)
        # one = np.ones(x_array.shape)
        # cos = np.cos(thetas).T
        # sin = np.sin(thetas).T
        #
        # T = np.concatenate((cos, -sin, x_array, sin, cos, y_array, zero, zero, one), axis = 1).reshape((3*len(point), 3))
        #
        # return T
