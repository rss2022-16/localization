import numpy as np
class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        pass

        ####################################

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
        
        ####################################
        # TODO

        # raise NotImplementedError
        particles = np.array(particles)
        shape = particles.shape
        # new_particles = np.zeros((shape[0], 3))
        dx = odometry[0]
        dy = odometry[1]
        dtheta = odometry[2]
        
        def compute_new_point(point):
        # for i in range(shape[0]):
            x = point[0]
            y = point[1]
            theta = point[2]
            new_x = (np.cos(theta) * dx) + (-np.sin(theta) * dy) + x
            new_y = (np.sin(theta) * dx) + (np.cos(theta) * dy) + y
            # new_theta = np.arccos(np.cos(theta)*np.cos(dtheta) + (-np.sin(theta) * np.sin(dtheta)))
            new_theta = theta + dtheta
            return [new_x, new_y, new_theta]

        new_particles = np.apply_along_axis(compute_new_point, 1, particles)
        return new_particles


        ####################################
