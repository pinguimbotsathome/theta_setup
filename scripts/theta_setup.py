#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


def pubTwist(initial_time):
    twist =  Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    while (rospy.get_rostime().to_sec() - initial_time.to_sec()) < 3.0: #3 segundos paradito aca
        pub.publish(twist)  
    rospy.logwarn("First message. Linear: %f  Angular: %f", twist.linear.x, twist.angular.z) 

if __name__ == '__main__':
    rospy.init_node('pubTwist', anonymous=True)
    initial_time = rospy.get_rostime()

    try:
        pubTwist(initial_time)
    except rospy.ROSInterruptException:
        pass    
