from shared_python_code.Point import CPoint 
from shared_python_code.Point import LOCATIONS
import math 


class GenerateAtNear(object):

    def __init__(self, near_ep, at_ep):
        self.near_ep = near_ep 
        self.at_ep = at_ep
        self.distance_dictionary = {}
        self.current_xy = None
        
    def epsilonFromLoc(self, distance, epsilon):
        if distance < epsilon:
            return True 
        return False 
    
    def distanceFromLocBetween(self, distance, upperBound, lowerBound):
        
        if(distance < upperBound):
            #print("{0} < {1}".format(distance,upperBound))
            if(distance >= lowerBound):
                #print("{0} >= {1}".format(distance,lowerBound))
                return True 
        return False         
 
    def euclideanDistance(self, cx, cy, lx, ly):
        distance = (cx - lx) * (cx - lx) + (cy - ly) * (cy - ly)
        distance = math.sqrt(distance)
        distance = math.fabs(distance)
        return distance
    
    def checkSetCurrentXY(self, cx, cy):
        if self.current_xy is None:
            self.current_xy = [cx, cy]
            self.generateDistanceDict()
            
        elif not (self.current_xy[0] == cx and self.current_xy[1] == cy):
            self.current_xy = [cx, cy]
            self.generateDistanceDict()
        # print(self.distance_dictionary)

    def generateDistanceDict(self):
        cx = self.current_xy[0]
        cy = self.current_xy[1]
        for loc in LOCATIONS:
            # get the distance from each location 
            lx = LOCATIONS[loc].x
            ly = LOCATIONS[loc].y
            self.distance_dictionary[loc] = self.euclideanDistance(cx, cy, lx, ly)
               
    def getAt(self, cx, cy):
        at_loc = None
        self.checkSetCurrentXY(cx, cy)
        for loc in LOCATIONS:
            distance = self.distance_dictionary[loc]
            if self.epsilonFromLoc(distance, self.at_ep):
                if at_loc is not None:
                    print("At two locations at once?? {0},{1} - going to update to {1}".format(at_loc, loc))
                at_loc = loc
                
        return at_loc
    
    def getNear(self, cx, cy):
        near_locs = []
        self.checkSetCurrentXY(cx, cy)
        for loc in LOCATIONS:
            distance = self.distance_dictionary[loc]
            #print("debug near locs -> {3} {0}, lb {1}, ub {2}".format(distance,self.near_ep,self.at_ep,loc))
            if self.distanceFromLocBetween(distance, self.near_ep, self.at_ep):
                near_locs.append(loc)
        #print("Near locs")
        #print (near_locs)        
        return near_locs

