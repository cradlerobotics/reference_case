#!/usr/bin/env python

# a simple visit

import rospy
from monitor_aggregator import MonitorAggregator

def mon_agg(spec_type,monlist):
    node = MonitorAggregator(monlist,"monitor_aggregator_{0}".format(spec_type))
    node.start_node()
    
def req_mon_agg():
    monlist = ['monitor_pg_radiation_visit_req_Fpipes', 'monitor_pg_radiation_visit_req_Fipone', 'monitor_pg_radiation_visit_req_Ft2bottom', 'monitor_pg_radiation_visit_req_Ftankset']
    mon_agg("req",monlist)
    
    

if __name__ == '__main__':

    try:
        req_mon_agg()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")
        
        
        

        

        
            


        


            
            
            
                
                
                
            
