import numpy as np
import math
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
        self.lidar_scale_to_map_scale = rospy.get_param("~lidar_scale_to_map_scale")

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
        self.map_resolution = None
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
        self.sensor_model_table = np.zeros((self.table_width, self.table_width), dtype=float)
        z_max = float(self.table_width - 1)
        index_array = np.where(self.sensor_model_table == 0)
        z = index_array[0] # z are rows
        d = index_array[1] # d is column

        z_float = z.astype(float)
        d_float = d.astype(float)

        # p_hit
        scalar = (1.0/((2.0*math.pi*self.sigma_hit**2)**0.5))
        exp = np.exp(-(np.square(z_float-d_float)/(2.0*self.sigma_hit**2)))
        p_hit = (scalar*exp).reshape((self.table_width, self.table_width))
        p_hit = p_hit/np.sum(p_hit, axis = 0, keepdims = True, dtype=float)
        self.sensor_model_table += self.alpha_hit*p_hit

        p_short = ((2.0/d_float)*(1.0-(z_float/d_float))).reshape((self.table_width, self.table_width))
        # p_short = 0 for d == 0 and z > d
        p_short[(z>d).reshape((self.table_width, self.table_width))] = 0.0
        p_short[:, 0] = 0.0
        # p_short[:, 1:] = p_short[:,1:]/np.sum(p_short[:,1:], axis = 0, keepdims = True)
        # p_short[:-1, :] = p_short[:-1,:]/np.sum(p_short[:-1,:], axis = 1, keepdims = True)
        self.sensor_model_table += self.alpha_short*p_short

        p_max = np.zeros((self.table_width, self.table_width), dtype=float)
        # p_max = 1 for last row 
        p_max[-1, :] = 1.0
        self.sensor_model_table += self.alpha_max*p_max

        p_rand = np.ones((self.table_width, self.table_width), dtype=float)/z_max
        self.sensor_model_table += self.alpha_rand*p_rand

        self.sensor_model_table = self.sensor_model_table/np.sum(self.sensor_model_table, axis = 0, keepdims = True, dtype=float) # columns sum to 1

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

        # scale distances from meters to pixels
        scalar = self.map_resolution*self.lidar_scale_to_map_scale
        scans = self.scan_sim.scan(particles)

        pixel_scans = scans/scalar
        pixel_observation = observation/scalar

        pixel_observation = np.clip(pixel_observation, 0, self.table_width-1)
        pixel_scans = np.clip(pixel_scans, 0, self.table_width-1)

        result = np.array(self.sensor_model_table)[pixel_observation.astype(int), pixel_scans.astype(int)]
        print(np.sum(result))
        return np.power(np.prod(result, axis = 1), 1.0/2.2)

        
        ####################################

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

        self.map_resolution = map_msg.info.resolution

        print("Map initialized")
