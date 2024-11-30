#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

def vins_to_mavros():
    rospy.init_node('vins_to_mavros')

    pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    pose_cov_pub = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseStamped, queue_size=10)
    velocity_pub = rospy.Publisher('/mavros/vision_speed/speed_vector', TwistStamped, queue_size=10)

    def vins_callback(odom):
        # Convert VINS Odometry to MAVROS-compatible messages
        pose = PoseStamped()
        pose_cov = PoseWithCovariance()
        velocity = TwistStamped()

        pose.header = odom.header
        pose.pose = odom.pose.pose

        pose_cov.header = odom.header
        pose_cov.pose = odom.pose

        velocity.header = odom.header
        velocity.twist = odom.twist.twist

        # Publish to MAVROS topics
        pose_pub.publish(pose)
        velocity_pub.publish(velocity)

    rospy.Subscriber('/ov_msckf/odomimu', Odometry, vins_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        vins_to_mavros()
    except rospy.ROSInterruptException:
        pass


