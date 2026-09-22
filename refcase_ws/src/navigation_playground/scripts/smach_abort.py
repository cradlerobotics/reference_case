#!/usr/bin/env python

import math
import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

from shared_python_code.Point import LOCATIONS, CPoint

# Arrival orientations for each safe location (radians, map frame).
# entrance: face -Y (south)  robot points away from corridor, ready to exit
# exit:     face +Y (north)  robot points into facility, ready to re-enter if needed
# refuge:   face -X (west)   robot points away from door, settled into refuge corner
WAYPOINT_YAW = {
    'entrance': -math.pi / 2,   # face south (-Y)
    'exit':      math.pi / 2,   # face north (+Y)
    'refuge':    math.pi,        # face west  (-X)
}

# ---------------------------------------------------------------------
# STATE: ON_ABORT
# ---------------------------------------------------------------------
class OnAbort(smach.State):
    def __init__(self, location_name, target_cpoint):
        smach.State.__init__(self, outcomes=['aborted', 'failed'])
        self.loc_name = location_name
        self.target_point = target_cpoint
        # Internal client for atomic operation
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo("--- ABORT SEQUENCE: Attempting to %s ---", self.loc_name)
        
        # 1. Check connection
        if not self.client.wait_for_server(timeout=rospy.Duration(2.0)):
            rospy.logerr("ON_ABORT: failed: move_base not available")
            return 'failed'

        # 2. Set Goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Using attributes from shared_python_code.Point object
        goal.target_pose.pose.position.x = self.target_point.x
        goal.target_pose.pose.position.y = self.target_point.y
        goal.target_pose.pose.position.z = self.target_point.z
        yaw = WAYPOINT_YAW.get(self.loc_name.lower(), 0.0)
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 3. Execute with shorter timeout for safety
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(timeout=rospy.Duration(60.0))

        if finished:
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Recovery Successful: Robot secure at %s", self.loc_name)
                return 'aborted'
        
        # 4. Fallthrough handling
        self.client.cancel_all_goals()
        rospy.logerr("Recovery to %s FAILED. Escalating...", self.loc_name)
        return 'failed'

# ---------------------------------------------------------------------
# STATE: SAFE HALT (Final Fallback)
# ---------------------------------------------------------------------
class SafeHalt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['halted'])
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.logerr("CRITICAL: All recovery attempts failed. ENGAGING SAFE HALT.")
        
        # 1. Force Stop
        stop_msg = Twist() # Zero velocity
        for i in range(10):
            self.cmd_pub.publish(stop_msg)
            rospy.sleep(0.1)

        # 2. Trigger External Safety System (Mocking the signal)
        rospy.logerr("SAFE HALT ACTIVE - MANUAL INTERVENTION REQUIRED")
        
        return 'halted'

# ---------------------------------------------------------------------
# MAIN EXECUTION
# ---------------------------------------------------------------------
def main():
    rospy.init_node('smach_abort_logic')

    rospy.loginfo("Initializing Abort Logic - Checking MoveBase...")
    temp_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    temp_client.wait_for_server()
    
    # Create SMACH
    sm_abort = smach.StateMachine(outcomes=['ABORTED', 'SYSTEM_HALTED'])

    with sm_abort:
        # PRIORITY 1: ENTRANCE
        smach.StateMachine.add('TRY_ENTRANCE', 
                               OnAbort('ENTRANCE', LOCATIONS['entrance']),
                               transitions={'aborted':'ABORTED', 
                                            'failed':'TRY_EXIT'})

        # PRIORITY 2: EMERGENCY EXIT
        smach.StateMachine.add('TRY_EXIT', 
                               OnAbort('EXIT', LOCATIONS['exit']),
                               transitions={'aborted':'ABORTED', 
                                            'failed':'TRY_REFUGE'})

        # PRIORITY 3: REFUGE
        smach.StateMachine.add('TRY_REFUGE', 
                               OnAbort('REFUGE', LOCATIONS['refuge']),
                               transitions={'aborted':'ABORTED', 
                                            'failed':'SAFE_HALT'})

        # PRIORITY 4: SAFE HALT (Emergency STOP)
        smach.StateMachine.add('SAFE_HALT', SafeHalt(),
                               transitions={'halted':'SYSTEM_HALTED'})

    sis = smach_ros.IntrospectionServer('abort_server', sm_abort, '/ABORT_SM')
    sis.start()

    outcome = sm_abort.execute()
    
    rospy.loginfo("Abort Sequence Final Outcome: " + outcome)
    sis.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass