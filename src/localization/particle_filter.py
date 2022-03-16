#!/usr/bin/env python2

import rospy
import tf2_ros as tf
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class ParticleFilter:

    def __init__(self):
        # Get parameters
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame")
        self.num_particles = rospy.get_param("~num_particles")

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.laser_cb, queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_cb, queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)

        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
        self.particles = np.empty([self.num_particles, 3])
        self.time_last_odom = rospy.Time.now()
        # self.best_particle = [0, 0, 0]

    def odom_cb(self, data):
        """
        """
        time = data.header.stamp
        linear = data.twist.twist.linear
        anglular = data.twist.twist.angular

        raw_odom = np.array([linear[0], linear[1], angular[2]])
        dt = time - self.time_last_odom
        self.time_last_odom = time
        odom = raw_odom * dt.to_sec()

        self.particles = self.motion_model.evaluate(self.particles, odom)

        self.update_avg_part()


    def laser_cb(self, data):
        """
        """
        ranges = data.ranges

        particle_probs = self.sensor_model.evaluate(self.particles, ranges)

        new_particle_idxs = np.random.choice(list(range(num_particles)), size=self.num_particles, replace=True, particle_probs)
        self.particles = [self.particles[idx] for idx in new_particle_idxs]

        self.update_avg_part()


    def init_cb(self, data):
        """
        """
        position = data.pose.position
        quaternion = data.pose.orientation
        angle = tf.transformations.euler_from_quaternion(quaternion) # check that angle in 2d plane

        self.particles = [[[position[0], position[1], angle[2]] for i in range(self.num_particles)]
        

    def update_avg_part(self):
        avg_x = sum([i[0] for i in self.particles])/self.num_particles
        avg_y = sum([i[1] for i in self.particles])/self.num_particles
        thetas = np.array([i[2] for i in self.particles])
        avg_theta = np.atan2(sum(np.sin(thetas)), sum(np.cos(thetas)))
        # self.best_particle = [avg_x, avg_y, avg_theta]

        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = avg_x
        odom_msg.pose.pose.position.y = avg_y
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, avg_theta)
        self.odom_pub.publish(odom_msg)



if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
