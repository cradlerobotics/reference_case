#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class MultiWallManager:
    def __init__(self):
        rospy.init_node('virtual_wall_pc_pub')
        
        # Topic used by SRAS for path planning and obstacle avoidance
        self.pub = rospy.Publisher('/virtual_obstacles', PointCloud2, queue_size=10, latch=True)
        
        # block entrance

        self.wall_configs = {
            'tankzone_block': {
                'x_range': (-6.2, 0.4), 
                'y_fixed': -3.51,
                'z': 0.1
            },
            'entrance_block_1': {
                'x_fixed': -6.5,
                'y_range': (-1.6, 0.515),
                'z': 0.1
            },
            'entrance_block_2': {
                'x_range': (-9, -6.5), 
                'y_fixed': 0.515,
                'z': 0.1
            }
        }

    def get_all_points(self):
        all_points = []
        resolution = 0.05  # 5cm density to satisfy planning discretisation

        for wall_name, conf in self.wall_configs.items():
            # Handle walls with fixed X (vertical segments)
            if 'x_fixed' in conf:
                curr_y = conf['y_range'][0]
                while curr_y <= conf['y_range'][1]:
                    all_points.append([conf['x_fixed'], curr_y, conf['z']])
                    curr_y += resolution
            
            # Handle walls with fixed Y (horizontal segments)
            elif 'y_fixed' in conf:
                curr_x = conf['x_range'][0]
                while curr_x <= conf['x_range'][1]:
                    all_points.append([curr_x, conf['y_fixed'], conf['z']])
                    curr_x += resolution
        
        return all_points

    def run(self):
        points = self.get_all_points()
        
        if points:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            
            # Create the PointCloud2 message containing all walls
            pc_msg = pc2.create_cloud_xyz32(header, points)
            rospy.loginfo("Publishing %d virtual points to /virtual_obstacles", len(points))
            self.pub.publish(pc_msg)
            rospy.spin()

if __name__ == '__main__':
    wall_manager = MultiWallManager()
    wall_manager.run()