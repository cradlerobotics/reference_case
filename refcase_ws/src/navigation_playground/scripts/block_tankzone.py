#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class VirtualWallManager:
    def __init__(self):
        rospy.init_node('virtual_wall_tankzone_pub')
        
        # Topic matches move_base configuration variable
        self.pub = rospy.Publisher('/virtual_obstacles', PointCloud2, queue_size=10, latch=True)
        
        # Define different partitions/wall configurations
        # Format: 'name': {'x_range': (start, end), 'y_range': (start, end), 'z': height}
        self.wall_configs = {
            'corridor_block': {
                'x_fixed': 3.0,
                'y_range': (-2.6, 0.9),
                'z': 0.1 # to stay in laser height
            },
            'tankzone_block': {
                # Vertically connects Entrance (approx x=1) to Tank1Face (approx x=5)
                'x_range': (-6.2, 0.4), 
                'y_fixed': -3.51,
                'z': 0.1
            }
        }

    def generate_points(self, config_name):
        points = []
        conf = self.wall_configs.get(config_name)
        
        if not conf:
            rospy.logerr("Wall configuration '%s' not found!", config_name)
            return points

        resolution = 0.05  # 5cm density to ensure no gaps for the planner

        # Case 1: Vertical wall along Y axis (Fixed X)
        if 'x_fixed' in conf:
            curr_y = conf['y_range'][0]
            while curr_y <= conf['y_range'][1]:
                points.append([conf['x_fixed'], curr_y, conf['z']])
                curr_y += resolution
        
        # Case 2: Horizontal wall along X axis (Fixed Y) - e.g., TankZone block
        elif 'y_fixed' in conf:
            curr_x = conf['x_range'][0]
            while curr_x <= conf['x_range'][1]:
                points.append([curr_x, conf['y_fixed'], conf['z']])
                curr_x += resolution

        return points

    def run(self):
        
        active_partition = 'tankzone_block' # Change this string to switch partitions easily
        
        points = self.generate_points(active_partition)
        
        if points:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            
            pc_msg = pc2.create_cloud_xyz32(header, points)
            rospy.loginfo("Publishing '%s' partition to /virtual_obstacles", active_partition)
            self.pub.publish(pc_msg)
            rospy.spin()

if __name__ == '__main__':
    wall_manager = VirtualWallManager()
    wall_manager.run()