import math

class CPoint(object):
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return "[x:{0},y:{1},z:{2}]".format(self.x,self.y,self.z)

    def __add__(self,other):
        x = self.x + other.x
        y = self.y + other.y
        z = self.z + other.z
        return CPoint(x,y,z)

    def __sub__(self,other):
        x = self.x - other.x
        y = self.y - other.y
        z = self.z - other.z
        return CPoint(x,y,z)

    def distance(self,other):
        dist = self-other
        squaredsum = dist.x*dist.x + dist.y*dist.y + dist.z*dist.z
        actualdist = math.sqrt(squaredsum)
        return actualdist


LOCATIONS = {
    'door': CPoint(0.0, 0.0, 0.0), 
    'entrance': CPoint(-3.8, 0.0, 0.0),
    'point1': CPoint(-5.09, 2.01,0.0),
    'point2': CPoint(-5.09, 5.69, 0.0),
	'point3': CPoint(-2.38, 5.69, 0.0),
	'point4': CPoint(-2.38, 2.5, 0.0),
	#'exit': CPoint(-6.61, 5.69, 0.0),
    'refuge': CPoint(-6.61, 3.76,0.0),
	'exit': CPoint(-4.69, 6.69, 0.0)
             }

class GwenHelper(object):
    
    def convert_to_gwen(self,locdict):
        namepred='location'
        locpred='location_coordinate'
        namelines = []
        pointlines = []
        for loc in locdict:
            namelines.append("{0}({1})".format(namepred,loc))
            pointlines.append("{pname}({lname},{x:.{digits}f},{y:.{digits}f},{z:.{digits}f})".format(pname=locpred,lname=loc,x=locdict[loc].x,y=locdict[loc].y,z=locdict[loc].z,digits=3))
            
        for names in namelines:
            print(names)
            
        for coords in pointlines:
            print (coords)


