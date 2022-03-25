# from nav_msgs.msg import Odometry.msg
import math
import numpy as np
import rospy

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        print ("ENTERED")
        

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
        result = np.apply_along_axis(self.get_next_pose, 1, particles, delta_x = odometry)
        return result

        ####################################

    def get_next_pose(self, old_x, delta_x):
        old_T= np.array([[math.cos(old_x[-1]), -math.sin(old_x[-1]), 0.0, old_x[0]], 
        [math.sin(old_x[-1]), math.cos(old_x[-1]), 0.0, old_x[1]], 
        [0.0,0.0,1.0, 0.0], 
        [0.0,0.0,0.0,1.0]])
        new_T= np.array([[math.cos(delta_x[-1]), -math.sin(delta_x[-1]), 0.0, delta_x[0]], 
        [math.sin(delta_x[-1]), math.cos(delta_x[-1]), 0.0, delta_x[1]], 
        [0.0,0.0,1.0, 0.0], 
        [0.0,0.0,0.0,1.0]])
        updated_T = np.dot(old_T,new_T)
        updated_theta = math.acos(updated_T[0,0])
        return np.array([updated_T[0,-1], updated_T[1, -1], updated_theta])


if __name__ == "__main__":
    rospy.init_node("motion_node")
    pf = MotionModel()
    rospy.spin()
