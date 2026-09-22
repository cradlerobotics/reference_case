#!/usr/bin/env python
# Publishes a red marker in RViz to indicate the trash can (discrepancy object)

import rospy
from visualization_msgs.msg import Marker

TRASH_CAN_X = 1.87 #1.87
TRASH_CAN_Y = -0.52
TRASH_CAN_Z = 0.0

def create_trash_can_marker():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time(0)
    marker.ns = "discrepancy_marker"
    marker.id = 0
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    marker.pose.position.x = TRASH_CAN_X
    marker.pose.position.y = TRASH_CAN_Y
    marker.pose.position.z = TRASH_CAN_Z + 0.35
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.7

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.8

    return marker

def create_text_marker():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time(0)
    marker.ns = "discrepancy_marker"
    marker.id = 1
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD

    marker.text = "discrepancy"

    marker.pose.position.x = TRASH_CAN_X - 1.0
    marker.pose.position.y = TRASH_CAN_Y
    marker.pose.position.z = TRASH_CAN_Z + 1.0
    marker.pose.orientation.w = 1.0

    marker.scale.z = 0.4

    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

if __name__ == "__main__":
    try:
        rospy.init_node('dm_marker_publisher')
        marker_pub = rospy.Publisher("dm_marker", Marker, queue_size=2)
        rate = rospy.Rate(1)

        trash_marker = create_trash_can_marker()
        text_marker = create_text_marker()

        while not rospy.is_shutdown():
            marker_pub.publish(trash_marker)
            marker_pub.publish(text_marker)
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("shutting down dm_marker_publisher")
