#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

def publish_pc_wall():
    pub = rospy.Publisher('/virtual_obstacles', PointCloud2, queue_size=10, latch=True)
    rospy.init_node('virtual_wall_pc_pub')
    
    x_pos = 3.0
    y_start = -2.6  # -0.85 center - (3.5/2)
    y_end = 0.9     # -0.85 center + (3.5/2)
    
    points = []
    # Create a dense line of points (every 5cm) to ensure no gaps
    curr_y = y_start
    while curr_y <= y_end:
        points.append([x_pos, curr_y, 0.1]) # Z=0.1 to stay in laser height
        curr_y += 0.05

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    
    # Create the PointCloud2 message
    pc_msg = pc2.create_cloud_xyz32(header, points)
    
    rospy.loginfo("Publishing PointCloud Wall at X=3.0 to block corridor...")
    pub.publish(pc_msg)
    rospy.spin()

if __name__ == '__main__':
    publish_pc_wall()