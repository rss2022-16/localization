#!/usr/bin/env python2

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
        self.particles = np.zeros([self.num_particles, 3])
        self.time_last_odom = rospy.Time.now()

        self.cloud_pub = rospy.Publisher("/cloud", PointCloud, queue_size = 1)

        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 10)


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


    def laser_cb(self, data):
        """
        """
        drive = AckermannDriveStamped()
        drive.header.stamp = rospy.Time.now()
        drive.header.frame_id = "base_link"
        drive.drive.steering_angle = 0.2
        drive.drive.speed = 0.25
        self.drive_pub.publish(drive)

        # ranges = np.array(data.ranges)

        # particle_probs = self.sensor_model.evaluate(self.particles, ranges)
        # prob_sum = np.sum(particle_probs)
        # particle_probs = particle_probs/prob_sum

        # new_particle_idxs = np.random.choice(list(range(self.num_particles)), size=self.num_particles, replace=True, p=particle_probs)
        # self.particles = np.array([self.particles[idx] for idx in new_particle_idxs])

        # self.update_avg_part()


    def init_cb(self, data):
        """
        """
        position = data.pose.pose.position
        quaternion = data.pose.pose.orientation
        angle = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        self.particles = np.array([[position.x, position.y, angle[-1]] for i in range(self.num_particles)])


    def update_avg_part(self):
        point_msg = PointCloud()
        point_msg.header.frame_id = "map"
        points = []
        for p in self.particles:
            point = Point32()
            point.x = 0 if abs(p[0]) < 1e-10 else float(p[0])
            point.y = 0 if abs(p[1]) < 1e-10 else float(p[1])
            point.z = 0
            points.append(point)
        point_msg.points = points
        self.cloud_pub.publish(point_msg)

        avg_x = sum([i[0] for i in self.particles])/self.num_particles
        avg_y = sum([i[1] for i in self.particles])/self.num_particles
        thetas = np.array([i[2] for i in self.particles])
        avg_theta = np.arctan2(sum(np.sin(thetas)), sum(np.cos(thetas)))
        
        rospy.loginfo("pred")
        rospy.loginfo([avg_x, avg_y, avg_theta])

        odom_msg = Odometry()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link_pf"
        odom_msg.pose.pose.position.x = avg_x
        odom_msg.pose.pose.position.y = avg_y
        odom_msg.pose.pose.position.z = 0
        angle = tf.transformations.quaternion_from_euler(0, 0, avg_theta)
        odom_msg.pose.pose.orientation.x = angle[0]
        odom_msg.pose.pose.orientation.y = angle[1]
        odom_msg.pose.pose.orientation.z = angle[2]
        odom_msg.pose.pose.orientation.w = angle[3]
        self.odom_pub.publish(odom_msg)



if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
