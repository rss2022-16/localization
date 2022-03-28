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
        xmu, xsig = 1, 4
        ymu, ysig = 1, 1
        tmu, tsig = 1, 4
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
        for i in range(len(particles)):

            if self.deterministic:

                odom_pose = self.vec2pose(odometry) # Deterministic odometry pose

            else:

                # Pick a random noise from our pre-computed gaussian samples
                noise = self.noise[np.random.randint(1, self.points)]
                # noisy_odom = np.matmul(odometry, np.diag(noise)) # Multiplying means proportional effec
                noisy_odom = np.transpose([odometry[0]*noise[0], odometry[1]*noise[1], odometry[2]*noise[2]])

                odom_pose = self.vec2pose(noisy_odom) # Noisy odometry pose
                
            # Compose particle pose and (noisy or deterministic) odometry pose into new particle pose
            particle_pose = self.vec2pose(particles[i])
            new_pose = np.matmul(particle_pose, odom_pose)
            new_particle = self.pose2vec(new_pose)
            new_particle[2] = particles[i,2] + odometry[2]

            # Can we do this in place or do we need to make a new array for new particles?
            particles[i] = new_particle

        return particles

    def vec2pose(self, point):
        """
        Given a 3-element vector [x, y, theta], returns the corresponding 3x3 pose matrix.
        """
        x_array = np.array([point[:, 0]]).T
        y_array = np.array([point[:,1]]).T
        thetas = np.array([point[:,-1]])
        zero = np.zeros(x_array.shape)
        one = np.ones(x_array.shape)
        cos = np.cos(thetas).T
        sin = np.sin(thetas).T

        T = np.concatenate((cos, -sin, x_array, sin, cos, y_array, zero, zero, one), axis = 1).reshape((3*len(point), 3))

        return T

    def sign(self, x):
        if x < 0:
            return -1
        return 1

    def pose2vec(self, pose):
        """
        Given a 3x3 pose matrix, returns the corresponding 3-element vector [x, y, theta].
        """
        # Arccosine loses sign information, need to check both sin and cos to tell which quadrant theta is in
        theta = np.arccos(pose[0,0]) * (self.sign(pose[1,0]))
        p = np.array([pose[0,2], pose[1,2], theta])

        return p
