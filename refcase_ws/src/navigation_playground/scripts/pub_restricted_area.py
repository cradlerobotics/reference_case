#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# Each area is defined by 4 corner points in CCW order (top-left, top-right, bottom-right, bottom-left)
# Original raw corners (any order):
#   area_1: [(0, -0.71), (0, -1.08), (2.56, -0.71), (2.56, -1.08)]
#   area_2: [(0.56, -3.94), (0.56, -5.0), (1.60, -3.94), (1.60, -5.0)]
#   area_3: [(3.27, -3.10), (3.27, -4.42), (4.13, -4.42), (4.13, -3.10)]
#   area_4: [(5.80, 0.8), (5.80, -2.45), (8.06, -2.45), (8.06, 0.8)]
# Sorted into CCW rectangle order: top-left, top-right, bottom-right, bottom-left

AREAS = [
    {
        'name': 'area_1',
        'corners': [
            (-0.45,  -0.71),
            (2.56, -0.71),
            (2.56, -1.30),
            (-0.45,  -1.30),
        ],
        'color': (1.0, 0.2, 0.2),  # red
    },
    {
        'name': 'area_2',
        'corners': [
            (0.56, -3.94),
            (1.60, -3.94),
            (1.60, -5.0),
            (0.56, -5.0),
        ],
        'color': (0.2, 1.0, 0.2),  # green
    },
    {
        'name': 'area_3',
        'corners': [
            (3.27, -3.10),
            (4.13, -3.10),
            (4.13, -4.42),
            (3.27, -4.42),
        ],
        'color': (0.2, 0.4, 1.0),  # blue
    },
    {
        'name': 'area_4',
        'corners': [
            (5.80,  0.8),
            (8.06,  0.8),
            (8.06, -2.45),
            (5.80, -2.45),
        ],
        'color': (1.0, 0.6, 0.0),  # orange
    },
]

Z_LEVEL = 0.01  # slightly above ground plane so it renders on top of the map


def make_fill_marker(marker_id, corners, color, frame_id, stamp):
    """TRIANGLE_LIST marker - transparent filled rectangle."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "restricted_area_fill"
    m.id = marker_id
    m.type = Marker.TRIANGLE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 1.0
    m.scale.y = 1.0
    m.scale.z = 1.0
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = 0.25  # transparent fill

    # Two triangles: (0,1,2) and (0,2,3)
    indices = [(0, 1, 2), (0, 2, 3)]
    for tri in indices:
        for idx in tri:
            p = Point()
            p.x = corners[idx][0]
            p.y = corners[idx][1]
            p.z = Z_LEVEL
            m.points.append(p)
    return m


def make_border_marker(marker_id, corners, color, frame_id, stamp):
    """LINE_STRIP marker - solid border around the rectangle."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "restricted_area_border"
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.04  # line width in metres
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = 0.9

    # Close the loop: corners[0..3] then back to corners[0]
    for idx in [0, 1, 2, 3, 0]:
        p = Point()
        p.x = corners[idx][0]
        p.y = corners[idx][1]
        p.z = Z_LEVEL
        m.points.append(p)
    return m


if __name__ == '__main__':
    rospy.init_node('restricted_area_publisher')
    pub = rospy.Publisher('restricted_areas', MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        ma = MarkerArray()
        stamp = rospy.Time.now()
        frame_id = "map"

        for i, area in enumerate(AREAS):
            corners = area['corners']
            color = area['color']
            fill_id = i * 2
            border_id = i * 2 + 1
            ma.markers.append(make_fill_marker(fill_id, corners, color, frame_id, stamp))
            ma.markers.append(make_border_marker(border_id, corners, color, frame_id, stamp))

        pub.publish(ma)
        rate.sleep()
