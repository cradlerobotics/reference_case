#!/usr/bin/env python

# just a class that checks whether a location has been reached or not
# so technically we're keeping track in terms of time
# after x seconds we've can exit the point


from Point import CPoint
from Point import LOCATIONS

#imported the point class ^

import math
from enum import Enum


class LocationProps(Enum):
    REACHED = 1,
    IN_LOC = 2,
    EXITED = 3,
    NONE = 4



class LocationReached(object):
    def __init__(self,loc,threshold):
        self.central_point = loc
        self.threshold = threshold
        self.loc_status = LocationProps.NONE
        self.reach_time = None


    def update_location_status(self,loc,t):
        dist = self.central_point.distance(loc)
        if dist < self.threshold:
            if self.reach_time is None:
                self.reach_time = t
                self.loc_status = LocationProps.REACHED
            else:
                if self.loc_status == LocationProps.EXITED:
                    self.reach_time = t
                    self.loc_status = LocationProps.REACHED
                else:
                    self.loc_status = LocationProps.IN_LOC
        else:
            if self.reach_time is not None:
                self.loc_status = LocationProps.EXITED

        return self.loc_status

    def has_reached_location(self,loc, t):
        self.update_location_status(loc,t)
        return (self.loc_status == LocationProps.REACHED)

    def is_in_location(self,loc,t):
        self.update_location_status(loc,t)
        return (self.loc_status == LocationProps.IN_LOC)

    def exited_location(self,loc, t):
        self.update_location_status(loc,t)
        return (self.loc_status == LocationProps.EXITED)


class LocationsReached(object):
    def __init__(self,threshold):
        self.locobjs = {}
        for loc in LOCATIONS:
            self.locobjs[loc]=LocationReached(LOCATIONS[loc],threshold)

    def get_loc_object(self,loc):
        return self.locobjs[loc]






