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
<<<<<<< HEAD
        xmu, xsig = 1, 3
        ymu, ysig = 1, 1
        tmu, tsig = 1, 3
=======
        xmu, xsig = 1, 0.5
        ymu, ysig = 0, 0.05
        tmu, tsig = 1, 0.5
>>>>>>> 29c798768518cc74ce2ebb95e44a22e460527088
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
