import this
import numpy as np
from math import sqrt, pi, exp
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")

        ####################################
        # TODO
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        self.z_max = 201
        ####################################

        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.
        
        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A
        
        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        self.sensor_model_table = np.fromfunction(lambda x, y: self.find_prob(x, y), \
                                                    (self.table_width, self.table_width), dtype=float)

        # normalizing along row (inefficient)
        # def normalize_row(self, row):
        #     sum = np.sum(row)
        #     return np.multiply(row, 1.0/sum)

        # self.sensor_model_table = np.apply_along_axis(normalize_row, 1, self.sensor_model_table)

        #normalize along row
        self.sensor_model_table = self.sensor_model_table / self.sensor_model_table.sum(axis = 1)
        #normalize along column
        self.sensor_model_table = self.sensor_model_table / self.sensor_model_table.sum(axis = 0)


    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 

        scans = self.scan_sim.scan(particles)

        ####################################


    def find_p_hit(self, z_k, d):
        if(z_k <= self.z_max):
            return 1.0/(sqrt(2*pi*self.sigma_hit**2))*exp(-(z_k-d)**2 / 2*self.sigma_hit**2)
        else:
            return 0.0
    
    def find_p_short(self, z_k, d):
        if(z_k <= d and d != 0):
            return 2.0/d * (1-z_k/d)
        else:
            return 0.0

    def find_p_max(self, z_k):
        return 1.0 if (self.z_max - 1 <= z_k and z_k <= self.z_max) else 0.0

    def find_p_rand(self, z_k):
        return 1.0/self.z_max if z_k <= self.z_max else 0.0

    # requires z_k to be <= z_k and >= 0
    def find_prob(self, z_k, d):
        return self.alpha_hit*self.find_p_hit(z_k, d) + self.alpha_short*self.find_p_short(z_k, d) \
                + self.alpha_max*self.find_p_max(z_k) + self.alpha_rand*self.find_p_rand(z_k)


    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
