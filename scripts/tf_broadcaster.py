#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import tf
import turtlesim.msg

def tf_broad(msg):
    br = tf.TransformBroadcaster()
    odom_trans = TransformStamped()
    odom_trans.header.stamp = msg.header.stamp
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_footprint"

    odom_trans.transform.translation.x = msg.pose.pose.position.x
    odom_trans.transform.translation.y = msg.pose.pose.position.y
    odom_trans.transform.translation.z = 0.0
    odom_trans.transform.rotation = msg.pose.pose.orientation 

    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0),
        (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) ,
        msg.header.stamp,
        "odom",
        "base_footprint")


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/odom', Odometry, tf_broad)
    while rospy.is_shutdown:
        print("teste")
        rospy.spin()