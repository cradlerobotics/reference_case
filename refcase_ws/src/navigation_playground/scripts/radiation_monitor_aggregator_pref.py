#!/usr/bin/env python

# a simple visit

import rospy
from monitor_aggregator import MonitorAggregator

def mon_agg(spec_type,monlist):
    node = MonitorAggregator(monlist,"monitor_aggregator_{0}".format(spec_type))
    node.start_node()
    

def pref_mon_agg():
    monlist=['monitor_pg_radiation_visit_pref_Gnotdangerred', 'monitor_pg_radiation_visit_pref_nott2bottomUipone', 'monitor_pg_radiation_visit_pref_nottanksetUipone', 'monitor_pg_radiation_visit_pref_notpipesUipone', 'monitor_pg_radiation_visit_pref_notpipesUt2bottom', 'monitor_pg_radiation_visit_pref_notpipesUtankset', 'monitor_pg_radiation_visit_pref_nottanksetUt2bottom']
    mon_agg("pref",monlist)
    

if __name__ == '__main__':

    try:
        pref_mon_agg()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")
        
        
        

        

        
            


        


            
            
            
                
                
                
            
