import rospy
from std_msgs.msg import String 

class MonitorAggregator(object):
    def __init__(self,monlist,aggmonname):
        #so we have a list of monitors
        #these are the names 
        #we listen to the verdicts for these 
        #so we subscribe to the verdicts of each 
        self.monlist = monlist
        self.sublist = {}
        self.verdictlist = {}
        for mon in self.monlist:
            
            self.sublist[mon]=rospy.Subscriber(mon+"/monitor_verdict", String,callback=self.monsub_callback,callback_args=mon)
        self.num_mons = len(self.monlist)
        self.publisher = rospy.Publisher("{0}/monitor_verdict".format(aggmonname),String,queue_size=10)
        self.name = aggmonname
        
            
            
    def monsub_callback(self,message,monname):
        self.verdictlist[monname] = message.data
        verdict_so_far = self.aggregate_verdicts()
        self.publisher.publish(verdict_so_far)
        
    def aggregate_verdicts(self):
        unknowns = self.sum_values(self.verdictlist, "unknown")
        trues = self.sum_values(self.verdictlist,"true")
        falses= self.sum_values(self.verdictlist, "false")
        if falses > 0:
            verdict_so_far = "false"
        elif trues == self.num_mons:
            verdict_so_far = "true"
        else:
            verdict_so_far = "unknown"

        return verdict_so_far
                
    
    def sum_values(self,dict,value):
        sumhere = 0 
        for key in dict:
            if dict[key] == value:
                sumhere = sumhere+1
                
        return sumhere
            
         
    def start_node(self):
        rospy.init_node(self.name)
        rospy.loginfo("Started node {0}".format(self.name))
        
        rospy.spin()
        
  
            
            