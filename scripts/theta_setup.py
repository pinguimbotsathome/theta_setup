#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub_pose = rospy.Publisher('/initial_2d', PoseStamped, queue_size=10)


def pubTwist(initial_time):
    twist =  Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    while (rospy.get_rostime().to_sec() - initial_time.to_sec()) < 3.0: #3 segundos paradito aca
        pub_vel.publish(twist)  
    rospy.logwarn("First message. Linear: %f  Angular: %f", twist.linear.x, twist.angular.z) 

def pubInitialPose():
	pose = PoseStamped()
	pose.header.seq = 0
	pose.header.frame_id = ''
	pose.pose.position.x = 0.0
	pose.pose.position.y = 0.0
	pose.pose.position.z = 0.0
	pose.pose.orientation.x = 0.0
	pose.pose.orientation.y = 0.0
	pose.pose.orientation.z = 0.0
	pose.pose.orientation.w = 0.0
	pub_pose.publish(pose)
	
	

if __name__ == '__main__':
    rospy.init_node('pubTwist', anonymous=True)
    initial_time = rospy.get_rostime()

    try:
        pubTwist(initial_time)
        pubInitialPose()
    except rospy.ROSInterruptException:
        pass    
