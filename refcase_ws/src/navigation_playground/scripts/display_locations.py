#!/usr/bin/env python
# displaying all locations 

import rospy
from shared_python_code.Point import CPoint 
from shared_python_code.Point import LOCATIONS
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class LocationMarkers(object):

    
    def __init__(self):
        rospy.init_node('location_markers')
        self.marker_pub = rospy.Publisher("visualization_marker_array",MarkerArray,queue_size=1)

    def create_loc_marker(self,point,m_id):
        marker = Marker()
        marker.header.frame_id="map"
        marker.header.stamp = rospy.Time(0)
        marker.ns = "locations"

        marker.type = Marker.SPHERE
        marker.id = m_id

        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.75

        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker

    def create_loc_text_marker(self,point,m_id,loc_name):
        marker = Marker()
        marker.header.frame_id="map"
        marker.header.stamp = rospy.Time(0)
        marker.ns = "location_names"

        marker.type = Marker.TEXT_VIEW_FACING
        marker.id = m_id

        marker.action = Marker.ADD

        marker.text = loc_name

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker

    
    def create_marker_array(self):
        locs = LOCATIONS
        locnum = 0
        markerArray = MarkerArray()
        
        for loc in locs:
            loc_pos = locs[loc]
            marker = self.create_loc_marker(loc_pos,locnum)
            #rospy.loginfo(marker)
            markerArray.markers.append(marker)
            marker_text = self.create_loc_text_marker(loc_pos,locnum+len(locs),loc)
            markerArray.markers.append(marker_text)
            locnum = locnum + 1

        return markerArray


    def publish_marker_array(self,markerArray):
        self.marker_pub.publish(markerArray)



    def publish_locations(self):
        markerArray = self.create_marker_array()
        #rospy.loginfo(markerArray)
        self.publish_marker_array(markerArray)


if __name__ == "__main__":

    try:
        locmarkers = LocationMarkers()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            locmarkers.publish_locations()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("shutting down")
        
        
        
            
            
            
        
        
        
