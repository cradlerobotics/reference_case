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
        
        # Define multiple partitions for the tankzone/entrance
        self.wall_configs = {
            'entrance_block_1': {
                'x_fixed': -7.8,
                'y_range': (-1.7, -0.5),
                'z': 0.1
            },
            'entrance_block_2': {
                'x_range': (-9.0, -7.8),
                'y_fixed': -0.5,
                'z': 0.1
            },
            'entrance_block_3': {
                'x_range': (-7.8, -6.6),
                'y_fixed': -1.7,
                'z': 0.1
            },
            'exit_block_1': {
                'x_fixed': 6.5,
                'y_range': (7.0, 9.4),
                'z': 0.1
            },
            'exit_block_2': {
                'x_range': (6.5, 9),
                'y_fixed': 7.0,
                'z': 0.1
            },
            'tankzone_block': {
                'x_range': (-6.2, 0.4), 
                'y_fixed': -3.51,
                'z': 0.1
            },
            'refuge_block_1': {
                'x_fixed': -6.0,
                'y_range': (7.0, 9.4),
                'z': 0.1
            },
            'refuge_block_2': {
                'x_range': (-9.0, -6.0),
                'y_fixed': 7.0,
                'z': 0.1
            }
        }

    def get_all_points(self):
        all_points = []
        resolution = 0.05  # 5cm density to satisfy planning discretisation

        for wall_name, conf in self.wall_configs.items():
            # Vertical line segment (fixed X)
            if 'x_fixed' in conf:
                curr_y = conf['y_range'][0]
                while curr_y <= conf['y_range'][1]:
                    all_points.append([conf['x_fixed'], curr_y, conf['z']])
                    curr_y += resolution

            # Horizontal line segment (fixed Y)
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