"""
Notes:
- Pre-compute cosines and sines
- How do we get "deterministic" and "num_particles" parameters into this file (for integration)
"""

import rospy
import numpy as np

class MotionModel:

    def __init__(self):

        self.deterministic = True # Pass in from particle_filter.py


        self.num_particles = 1000 # Pass in from particle_filter.py
        self.particles = np.empty([self.num_particles, 3])

        # Pre-compute cosines, sines, arcsines, and arccosines:
            # Every time we run vec2pose and pose2vec it uses cosines, sines, arccosines, and arcsines
            # So we should make a map of theta --> costheta, and costheta --> theta, etc.
            # Currently not implemented -- just using np.sin, np.cos, np.arccos, np.arcsin in helper functions
            # Is this worth our time?

        # Pre-computing gaussian noise
        # Sigma values are the parameters we tune!
        xmu, xsig = 1, 0.1
        ymu, ysig = 1, 0.025
        tmu, tsig = 1, 0.1
        self.points = 1000
        self.noise = np.random.normal([xmu, ymu, tmu], [xsig, ysig, tsig], size = (self.points, 3))
        print self.noise

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
        rospy.loginfo(particles[0])

        for i in range(len(particles)):

            if self.deterministic:

                odom_pose = self.vec2pose(odometry) # Deterministic odometry pose

            else:

                # Pick a random noise from our pre-computed gaussian samples
                noise = self.noise[np.random.randint(1, self.points)]
                noisy_odom = odometry * np.diag(noise) # Multiplying means proportional effect

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
        T = np.array(
            [[np.cos(point[2]), -np.sin(point[2]), point[0]],
            [np.sin(point[2]), np.cos(point[2]), point[1]],
            [0, 0, 1]])

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
