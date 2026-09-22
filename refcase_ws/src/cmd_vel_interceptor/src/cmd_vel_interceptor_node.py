#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelInterceptor:
    def __init__(self):
        rospy.init_node('cmd_vel_interceptor_node', anonymous=True)
        
        # Internal state 
        self.stop_requested = False # Stop signal

        # Subscribers 
        #rospy.Subscriber('jackal_velocity_controller/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/gwendolen_control', Bool, self.stop_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_filtered', Twist, queue_size=10)
        
        rospy.loginfo("cmd_vel_interceptor_node initialized and running...")

    def stop_callback(self, msg):
        self.stop_requested = msg.data
        rospy.loginfo("Safety stop signal received!")
        #rospy.loginfo(f"Safety stop signal received: {self.stop_requested}")

    def cmd_vel_callback(self, msg):
        if self.stop_requested:
            rospy.logwarn("Stop is active - publishing zero velocity")
            self.cmd_vel_pub.publish(Twist())  # Publish zero velocities
        else:
            self.cmd_vel_pub.publish(msg)  # Forward original message

if __name__ == '__main__':
    try:
        interceptor = CmdVelInterceptor()
        rospy.spin()      # Blocks until ROS node is shutdown
    except rospy.ROSInterruptException:
        rospy.loginfo("cmd_vel_interceptor_node shutting down.")
