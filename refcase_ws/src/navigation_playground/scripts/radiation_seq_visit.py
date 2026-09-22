#!/usr/bin/env python

import rospy
from SimpleVisit import SimpleVisit


if __name__ == '__main__':

    try:
        locs = ['exit']
        simple_visit = SimpleVisit()
        simple_visit.start(locs)

    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")
        
        