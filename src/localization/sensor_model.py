import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt

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
        # raise NotImplementedError
        z_max= self.table_width-1
        self.sensor_model_table = np.zeros((self.table_width, self.table_width))
        for j in range(self.table_width): # d values
            prob_hit_sum = 0.
            # norm_sum = 0.
            prob_hit_list = np.zeros(self.table_width)
            # prob_hit_list = []
            for i in range(self.table_width): # z values
                prob_hit = (1./(self.sigma_hit * np.sqrt(2.*np.pi))) * np.exp(-(float(i- j)**2)/(2.*(self.sigma_hit**2)))
                # prob_hit = np.exp(-((i-j)**2.)/(2.*self.sigma_hit**2.))
                prob_hit_sum += prob_hit
                prob_hit_list[i] = prob_hit
                # prob_hit_list.append(prob_hit)
            # for i in range(self.table_width):
                # prob_hit = self.alpha_hit * (prob_hit_list[i]/prob_hit_sum)
                if i <= j and j!= 0:
                    # prob_short = self.alpha_short * (2./float(j)) * (1.-(float(i/j)))
                    prob_short = 2.0 * self.alpha_short * (j - i) / float(j ** 2)
                else:
                    prob_short = 0.
                if i == z_max:
                    prob_max = self.alpha_max 
                else:
                    prob_max = 0.
                if i <= z_max:
                    prob_rand = self.alpha_rand * (1./float(z_max))
                else:
                    prob_rand = 0.
                prob = prob_short + prob_max + prob_rand
                # norm_sum += prob
                self.sensor_model_table[i][j] = prob
            # Deal with hit
            prob_hit = self.alpha_hit * (prob_hit_list/prob_hit_sum)
            self.sensor_model_table[:, j] += prob_hit
            # norm_sum += self.alpha_hit
            # self.sensor_model_table[:, j] /= norm_sum
            
        # normalize columns
        print(self.sensor_model_table)
        print(self.sensor_model_table.sum(axis = 0, keepdims = 1))
        self.sensor_model_table = self.sensor_model_table/self.sensor_model_table.sum(axis = 0, keepdims = 1)

        # plt.plot(self.sensor_model_table)
        print(self.sensor_model_table.sum(axis = 0, keepdims = 1))
            # for k in range(self.table_width):
            #     column_sum = np.sum(self.sensor_model_table, axis = 0)

        
        # # raise NotImplementedError
        # z_max= self.table_width-1
        # self.sensor_model_table = np.zeros((self.table_width, self.table_width))
        # for i in range(self.table_width): # z values
        #     prob_hit_sum = 0.
        #     prob_hit_list = []
        #     for j in range(self.table_width): # d values
        #         prob_hit = (1./np.sqrt(2.*np.pi*(self.sigma_hit**2.))) * np.exp(-((float(i)-float(j))**2.)/(2.*self.sigma_hit**2.))
        #         # prob_hit = np.exp(-((i-j)**2.)/(2.*self.sigma_hit**2.))
        #         prob_hit_sum += prob_hit
        #         prob_hit_list.append(prob_hit)
        #     for j in range(self.table_width):
        #         prob_hit = self.alpha_hit * (prob_hit_list[j]/prob_hit_sum)
        #         if i <= j and j!= 0:
        #             prob_short = self.alpha_short * (2./float(j)) * (1.-(float(i)/float(j)))
        #         else:
        #             prob_short = 0.
        #         if i == z_max:
        #             prob_max = self.alpha_max 
        #         else:
        #             prob_max = 0.
        #         prob_rand = self.alpha_rand * (1./float(z_max))
        #         prob = prob_hit + prob_short + prob_max + prob_rand
        #         self.sensor_model_table[i][j] = prob
            
        # # normalize columns
        # print(self.sensor_model_table)
        # print(self.sensor_model_table.sum(axis = 0, keepdims = 1))
        # self.sensor_model_table = self.sensor_model_table/self.sensor_model_table.sum(axis = 0, keepdims = 1)
        # print(self.sensor_model_table.sum(axis = 0, keepdims = 1))
        #     # for k in range(self.table_width):
        #     #     column_sum = np.sum(self.sensor_model_table, axis = 0)


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

        # lidar_scale_to_map_scale = rospy.get_param("~lidar_scale_to_map_scale")
        # scans = scans/(self.map_resolution*lidar_scale_to_map_scale)
        # observation = observation/(self.map_resolution*lidar_scale_to_map_scale)

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

        print("Map initialized")

if __name__ == '__main__':
    SensorModel()
