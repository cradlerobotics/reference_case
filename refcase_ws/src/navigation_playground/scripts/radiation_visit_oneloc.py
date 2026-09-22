#!/usr/bin/env python

# a simple visit

import rospy
from SimpleVisit import SimpleVisit



if __name__ == '__main__':

    try:
        locs = ['entrance', 'point1']
        simple_visit = SimpleVisit()
        simple_visit.start(locs)

    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")
        
        
        

        

        
            


        


            
            
            
                
                
                
            
