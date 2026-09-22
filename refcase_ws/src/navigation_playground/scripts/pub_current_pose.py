#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import TransformListener


class PosePub:
    def __init__(self):
        self.pose = Vector3()
        self.sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.get_pose)
        self.pub = rospy.Publisher("current_pose", Vector3, queue_size=10)
        rospy.init_node("pub_current_pose", anonymous=True)
        rospy.loginfo("Started pose node")
        self.tf = TransformListener()
        self.rate = rospy.Rate(10)
        rospy.loginfo("Publishing pose from amcl_pose to current_pose")


    def get_pose(self, data):
        #rospy.loginfo(data)
        self.pose = Vector3()
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        self.pose.z = data.pose.pose.position.z
        # rospy.loginfo("pose")
        # rospy.loginfo(self.pose)
        # rospy.loginfo("callback finished")

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.pose)
            # rospy.loginfo(self.pose)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        pp = PosePub()
        pp.run()
    except rospy.ROSInterruptException:
        pass
