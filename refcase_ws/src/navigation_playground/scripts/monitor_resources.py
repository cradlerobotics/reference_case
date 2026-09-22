#!/usr/bin/env python

# this file has all the monitor resources 
# because I dont want to worry about launch files etc 
# this is obviously not the best way to do it 
# but I think it works 

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_radiation_plugins.msg import Simulated_Radiation_Msg
from std_msgs.msg import String
from shared_python_code.GenerateAtNear import GenerateAtNear

class MonitorSub:
    
    def __init__(self):
        self.predicateGenerator=GenerateAtNear(near_ep=3,at_ep=0.5)
        self.rad_sub = rospy.Subscriber("/radiation_sensor_plugin/sensor_0",Simulated_Radiation_Msg,self.rad_callback)
        self.pose_sub = rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.pose_callback)
        self.rad_status_pub = rospy.Publisher("radiation_status",String,queue_size= 10)
        self.pos_loc_pub = rospy.Publisher("at_location",String,queue_size=10)
        self.pos_loc_near_pub = rospy.Publisher("near_locations",String,queue_size=10)
        self.radiation_value = None
        self.rad_status = None
        self.at_pred = None 
        self.near_preds = None
        
        self.start_node()

    
    def start_node(self):
        rospy.init_node("useful_topics_to_predicates")
        rospy.loginfo("Started node that converts radiation levels to status and pose information to at, near")
        self.rate = rospy.Rate(10)
        
    def rad_callback(self,data):
        if(self.radiation_value is None):
            self.radiation_value = data.value
        else:
            self.radiation_value = (self.radiation_value+data.value)/2
        if(self.radiation_value>=120):
            self.rad_status = "danger_red";
        elif self.radiation_value >=90:
            self.rad_status = "danger_orange"
        else:
            self.rad_status = "green"
            
    def pose_callback(self,data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y 
        z = data.pose.pose.position.z
        self.at_pred = self.predicateGenerator.getAt(x,y)
        self.near_preds = self.predicateGenerator.getNear(x,y)
        #print(self.near_preds)
        
    def run(self):
        while not rospy.is_shutdown():
            
            if self.near_preds is not None and len(self.near_preds)!=0:
                nlocs = ','.join(self.near_preds)
                self.pos_loc_near_pub.publish(nlocs)
                rospy.loginfo(nlocs)
            if self.at_pred is not None:
                self.pos_loc_pub.publish(self.at_pred)
                rospy.loginfo(self.at_pred)
            if self.rad_status is not None: 
                self.rad_status_pub.publish(self.rad_status)
                rospy.loginfo(self.rad_status)
            self.rate.sleep()
        
            
            
           

if __name__ == '__main__':
    try:
        pp = MonitorSub()
        pp.run()
            
        
    except rospy.ROSInterruptException:
        pass
