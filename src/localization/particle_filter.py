#!/usr/bin/env python2

"""
Note:
For keyop, use rosrun ackermann_drive_teleop keyop.py [max_speed] [max_steer] [topic name]
"""

import rospy
import tf
# import tf2_ros as tf
import numpy as np
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point32

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import PointCloud

from std_msgs.msg import Float32


class ParticleFilter:

    def __init__(self):
        # Get parameters
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame")
        self.num_particles = rospy.get_param("~num_particles")
        self.num_beams = rospy.get_param("~num_beams_per_particle")
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
        self.particles = np.zeros([self.num_particles, 3])
        self.time_last_odom = rospy.Time.now()

        self.cloud_msg = PointCloud()
        self.cloud_msg.header.frame_id = "map"

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = self.particle_filter_frame

        self.cloud_pub = rospy.Publisher("/cloud", PointCloud, queue_size = 1)

        self.error_pub = rospy.Publisher("/error", Float32, queue_size = 1)
        self.x = 0
        self.y = 0

        # Initialize Broadcaster


    def odom_cb(self, data):
        """
        """
        time = data.header.stamp
        linear = data.twist.twist.linear
        angular = data.twist.twist.angular

        raw_odom = np.array([linear.x, linear.y, angular.z])
        dt = time - self.time_last_odom
        self.time_last_odom = time
        odom = raw_odom * dt.to_sec()

        self.particles = self.motion_model.evaluate(self.particles, odom)

        self.update_avg_part()

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y


    def laser_cb(self, data):
        """
        """
        ranges = np.array(data.ranges)
        ranges = [ranges[i] for i in np.linspace(0,1080,num=self.num_beams,dtype=int)]
        ranges = np.array(ranges)
        # ^^ Downsampling to 100 beams
        particle_probs = self.sensor_model.evaluate(self.particles, ranges)
        prob_sum = np.sum(particle_probs)
        particle_probs = particle_probs/prob_sum

        new_particle_idxs = np.random.choice(list(range(self.num_particles)), size=self.num_particles, replace=True, p=particle_probs)
        self.particles = np.array([self.particles[idx] for idx in new_particle_idxs])

        # self.update_avg_part()


    def init_cb(self, data):
        """
        """
        position = data.pose.pose.position
        quaternion = data.pose.pose.orientation
        angle = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        self.particles = np.array([[position.x, position.y, angle[-1]] for i in range(self.num_particles)])

        self.x = position.x
        self.y = position.y

    def update_avg_part(self):
        points = []
        for p in self.particles:
            point = Point32()
            point.x = 0 if abs(p[0]) < 1e-10 else float(p[0])
            point.y = 0 if abs(p[1]) < 1e-10 else float(p[1])
            point.z = 0
            points.append(point)
        self.cloud_msg.points = points
        self.cloud_pub.publish(self.cloud_msg)

        avg_theta = np.arctan2(np.sum(np.sin(self.particles[:,2])), np.sum(np.cos(self.particles[:,2])))
        avg_x = np.average(self.particles[:,0]) - np.cos(avg_theta)*.275
        avg_y = np.average(self.particles[:,1]) - np.sin(avg_theta)*.275
        
        # Publish transform
        
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = avg_x
        self.odom_msg.pose.pose.position.y = avg_y
        self.odom_msg.pose.pose.position.z = 0
        angle = tf.transformations.quaternion_from_euler(0, 0, avg_theta)
        self.odom_msg.pose.pose.orientation.x = angle[0]
        self.odom_msg.pose.pose.orientation.y = angle[1]
        self.odom_msg.pose.pose.orientation.z = angle[2]
        self.odom_msg.pose.pose.orientation.w = angle[3]
        self.odom_pub.publish(self.odom_msg)

        msg = Float32()
        msg.data = np.sqrt( (self.x - avg_x)**2 + (self.y - avg_y)**2 )
        self.error_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
