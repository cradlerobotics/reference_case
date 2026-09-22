#!/usr/bin/env python
"""
map_cropper.py

Subscribes to a (potentially large) /map OccupancyGrid and republishes a
cropped sub-region as /map_nav.  move_base static_layer subscribes to
/map_nav, keeping the full map available to AMCL without crashing move_base
with a ros::serialization::StreamOverrunException on large maps.

Crop region is read from ROS params (should match global_costmap_params.yaml):
  ~crop_origin_x   (default -6.0)
  ~crop_origin_y   (default -6.0)
  ~crop_width      (default 12.0)   metres
  ~crop_height     (default 12.0)   metres
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid


class MapCropper(object):
    def __init__(self):
        rospy.init_node('map_cropper')

        self.crop_ox = rospy.get_param('~crop_origin_x', -6.0)
        self.crop_oy = rospy.get_param('~crop_origin_y', -6.0)
        self.crop_w  = rospy.get_param('~crop_width',    12.0)
        self.crop_h  = rospy.get_param('~crop_height',   12.0)

        self.pub = rospy.Publisher('/map_nav', OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)

        rospy.loginfo("map_cropper: crop region x=[%.1f, %.1f] y=[%.1f, %.1f]",
                      self.crop_ox, self.crop_ox + self.crop_w,
                      self.crop_oy, self.crop_oy + self.crop_h)

    def map_callback(self, msg):
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        # Crop bounds in grid cells
        gx_min = int((self.crop_ox - ox) / res)
        gy_min = int((self.crop_oy - oy) / res)
        gx_max = int((self.crop_ox + self.crop_w - ox) / res)
        gy_max = int((self.crop_oy + self.crop_h - oy) / res)

        # Clamp to source map bounds
        gx_min = max(0, gx_min)
        gy_min = max(0, gy_min)
        gx_max = min(msg.info.width,  gx_max)
        gy_max = min(msg.info.height, gy_max)

        if gx_max <= gx_min or gy_max <= gy_min:
            rospy.logerr("map_cropper: crop region outside source map bounds - check params")
            return

        full = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        sub  = full[gy_min:gy_max, gx_min:gx_max]

        out = OccupancyGrid()
        out.header = msg.header
        out.info.map_load_time   = msg.info.map_load_time
        out.info.resolution      = res
        out.info.width           = gx_max - gx_min
        out.info.height          = gy_max - gy_min
        out.info.origin.position.x = self.crop_ox
        out.info.origin.position.y = self.crop_oy
        out.info.origin.orientation.w = 1.0
        out.data = sub.flatten().tolist()

        self.pub.publish(out)
        rospy.loginfo_once("map_cropper: published /map_nav (%dx%d cells, %.1fx%.1fm)",
                           out.info.width, out.info.height, self.crop_w, self.crop_h)


if __name__ == '__main__':
    node = MapCropper()
    rospy.spin()
