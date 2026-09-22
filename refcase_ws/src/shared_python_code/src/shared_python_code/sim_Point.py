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


LOCATIONS = {'door':CPoint(-8.18,-3.16,0.0), 'drumpipes': CPoint(-4.07,-7.24,0.0),
             'drum':CPoint(-7.25,-5.26,0.0), 'tank1face': CPoint(-0.49,-3.51,0.0),
             'tank1': CPoint(2.30,-1.93,0.0), 'tank1pipe': CPoint(5.48,-3.48,0.0),
             'tank2face': CPoint(-0.47,-6.52,0.0), 'tank2': CPoint(1.98,-7.88,0.0),
             'tank2pipe': CPoint(5.36,-6.31,0.0), 'tankset': CPoint(6.80,-5.69,0.0),
             'pipes': CPoint(7.79,-2.67,0.0), 'room': CPoint(1.30,2.20,0.0),
             'stairs': CPoint(2.94,8.56,0.0),
             'entrance':CPoint(-7.6, -0.8, 0.0),
             'refuge':CPoint(-7.0, 7.8, 0.0),
             'exit': CPoint(7.74552, 7.8, 0.0),
             'roomwall':CPoint(-0.5568475723266602, 0.515995979309082, 0.00116729736328125),
             'roomdoor':CPoint(4.197497367858887, 0.3372831344604492, 0.004076957702636719),
             'roomback':CPoint(4.948063850402832, 4.055242538452148, 0.0),
             'tankzone':CPoint(  -2.1981678009033203, -4.938733100891113, 0.0),
             'freezone':CPoint( -5.969668388366699, 3.861062526702881, 0.0),
             'farwall':CPoint( 7.745523452758789,3.8347959518432617,0.0),
             'corridor':CPoint(1.8702278137207031,-0.5160531997680664,0.0)
             }

#'exit':CPoint(-7.8254, 8.56, 0.0),
#'refuge': CPoint(7.74552, 8.56, 0.0), 

#cuz I dont want to have to do this again and again 
#takes locations in the format above 
#outputs location(name) and location_coordinates(name,x,y,z) as a list 

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


