#! /usr/bin/env python3

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Vector3

class movebase_client:
    def __init__(self,x,y,z,done_cb):
        self.client = self.init_client_and_send_to_movebase(x,y,z,done_cb)
        
        
    def init_client_and_send_to_movebase(self,x,y,z,done_cb):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        wait = client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Action server available")           

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal,done_cb)
        rospy.loginfo("Goal sent - Location ({0},{1},{2})".format(x,y,z))
        return client
    
    def cancel_goal(self):
        if self.client is not None:
            self.client.cancel_goal()
            rospy.loginfo("Cancelled goal!")
            
    
class gwendolen_to_movebase:
    
    def __init__(self):
        self.mbclient = None 
        self.movebase_client()
    
    
    def is_stop_goal(self,data):
        
        if(data.x==0 and data.y==0 and data.z == 0):
            return True
        return False
    
     
    def done_callback(self,status,result):
        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Goal received a cancel request after it started execution")
            
        elif status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached")
    
            

        elif status == GoalStatus.ABORTED:
            rospy.loginfo("Goal aborted")


        elif status == GoalStatus.REJECTED:
            rospy.loginfo("Goal rejected")


        elif status == GoalStatus.RECALLED:
            rospy.loginfo("Goal received a cancel request before execution. Cancelled!")
        self.mbclient = None
     
    def goal_movebase(self,data):
    	
        try: 
            if not self.is_stop_goal(data):
                if self.mbclient is not None:
                    rospy.logerr("Error - sending a goal when one is already in execution, this should not happen")
                self.mbclient = movebase_client(data.x,data.y,data.z,self.done_callback)
            else:
                if self.mbclient is not None:
                    self.mbclient.cancel_goal()
                else:
                    rospy.logerr("Unable to stop current goal, seems its done already")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")    
    
    def movebase_client(self):
    	
    	rospy.init_node('gwendolen_to_ros_movebase_client_py', anonymous=True)
    	
    	rospy.Subscriber("gwendolen_to_move_base", Vector3, self.goal_movebase)
    	
    	rospy.spin()

if __name__ == '__main__':
    gwendolen_to_movebase()
