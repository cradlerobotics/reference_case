#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

def publish_virtual_wall(marker_id, x_pos, y_pos, length_x, width_y, frame_id="map"):
    pub = rospy.Publisher('virtual_obstacles', Marker, queue_size=10, latch=True)
    wall = Marker()
    wall.header.frame_id = frame_id
    wall.header.stamp = rospy.Time.now()
    wall.ns = "virtual_walls"
    wall.id = marker_id
    wall.type = Marker.CUBE
    wall.action = Marker.ADD
    
    wall.pose.position.x = x_pos
    wall.pose.position.y = y_pos
    wall.pose.position.z = 0.5
    wall.pose.orientation.w = 1.0 # Fixes the uninitialized quaternion error
    
    wall.scale.x = length_x 
    wall.scale.y = width_y
    wall.scale.z = 1.0 
    
    # color red
    # wall.color.r = 1.0; wall.color.g = 0.0; wall.color.b = 0.0; wall.color.a = 0.8
    wall.color.r = 0.5; wall.color.g = 0.5; wall.color.b = 0.5; wall.color.a = 1.0
    pub.publish(wall)

# Scenario 1: Re-partitioning to block the central corridor
# Forces failure during mission visit to tankzone
def partition_scenario_one():
    # Blocks the hallway between Freezone and Tankzone
    publish_virtual_wall(marker_id=1, x_pos=1.5, y_pos=-0.5, length_x=0.5, width_y=4.0)

# Scenario 2: Re-partitioning to block the Entrance
# Forces recovery logic to switch from ENTRANCE to EXIT
def partition_scenario_two():
    # Encapsulates the entrance point in a forbidden block
    ent = LOCATIONS['entrance']
    publish_virtual_wall(marker_id=2, x_pos=ent.x, y_pos=ent.y, length_x=2.0, width_y=2.0)
    

if __name__ == '__main__':
    rospy.init_node('virtual_wall_publisher')
    rate = rospy.Rate(1)
    
    # Scenario 1: Block the corridor (coords)
    # x_pos, y_pos, length_x, width_y
    s1_params = [6.5, 10.0, 3.0, 0.2] 

    # x_pos, y_pos, length_x, width_y
    s2_params = [1.0, 1.0, 1.5, 1.5]

    while not rospy.is_shutdown():
        
        # Test Scenario 1: Path to next goal fails
        # publish_virtual_wall(1, *s1_params) 
        # Scenario: Horizontal block at the roomwall/corridor gateway level
        # Forces plan_fail when trying to navigate from Freezone to Tankzone
        publish_virtual_wall(marker_id=3, x_pos=3.0, y_pos=-0.85, length_x=0.2, width_y=3.5)
    
        # Test Scenario 2: Path to Entrance fails (Run during Abort)
        # publish_virtual_wall(2, *s2_params) 
        
        rate.sleep()