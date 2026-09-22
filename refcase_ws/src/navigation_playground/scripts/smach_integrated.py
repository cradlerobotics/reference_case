#!/usr/bin/env python

import rospy
import math
import smach
import smach_ros
import actionlib
import tf2_ros
import json
import os

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalID
import dynamic_reconfigure.client

from shared_python_code.Point import LOCATIONS, CPoint
from shared_python_code.LocReached import LocationReached, LocationProps

audit_event_pub = None

class RunTracker:
    def __init__(self):
        self.t1 = {
            'Planning':          {'entered': 0, 'plan_ok': 0, 'plan_fail': 0, 'hazard': 0},
            'AbortPlanEntrance': {'entered': 0, 'plan_ok': 0, 'plan_fail': 0, 'hazard': 0},
            'AbortPlanExit':     {'entered': 0, 'plan_ok': 0, 'plan_fail': 0, 'hazard': 0},
            'AbortPlanRefuge':   {'entered': 0, 'plan_ok': 0, 'plan_fail': 0, 'hazard': 0}
        }
        self.t2 = {
            'ExecConstraint':      {'entered': 0, 'arrived': 0, 'stuck': 0, 'hazard': 0},
            'ExecPointGo':         {'entered': 0, 'arrived': 0, 'stuck': 0, 'hazard': 0},
            'AbortExecConstraint': {'entered': 0, 'arrived': 0, 'stuck': 0, 'hazard': 0},
            'AbortExecPointGo':    {'entered': 0, 'arrived': 0, 'stuck': 0, 'hazard': 0}
        }
        self.t3 = {
            'Inspect': {'entered': 0, 'inspect_ok': 0, 'hazard': 0}
        }
        
        self.t4 = {'Monitoring': {'count_geofenceViolation': 0, 'time_spent_in_this_state': 0}, 
                   'WaitForHalt': {'count_geofenceViolation': 0, 'time_spent_in_this_state': 0}, 
                   'SafeHaltActive': {'count_geofenceViolation': 0, 'time_spent_in_this_state': 0}}
        self.t5 = {'WaitForHalt': {'count_haltObserved': 0, 'count_tick': 0, 'Total_number_of_times_WaitForHalt': 0}}
        self.t6 = {'SafeHaltActive': {'count_move': 0, 'Total_number_of_times_SafeHaltActive': 0}}

        self.min_dist_observed = float('inf')
        self.threshold_geofence = 0.45
        self.timestamp_geofence_violation = None
        self.timestamp_enforced_stop = None

        self.m13_episodes = []
        self.active_halt_request = None 

        self.current_state = None
        self.current_state_start = None
        self.enforced_stop_triggered = False
        self.event_log = []

    def record_scan(self, msg):
        """M12/M14: Track min distance and detect physical breach for M14 latency."""
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            current_min = min(valid)
            if current_min < self.min_dist_observed:
                self.min_dist_observed = current_min
            
            if current_min <= self.threshold_geofence and self.timestamp_geofence_violation is None:
                self.timestamp_geofence_violation = rospy.Time.now()
                rospy.logwarn("[M14] Geofence Threshold Breached (%.3fm). Timer started.", current_min)

    def record_enforced_stop_time(self):
        """M14 End: Captures the time Gwendolen state entered EnforcedStop."""
        if self.timestamp_enforced_stop is None:
            self.timestamp_enforced_stop = rospy.Time.now()

    def start_halt_request_timer(self):
        """Starts the M13 timer the moment SMACH initiates a Safe Halt."""
        if self.active_halt_request is None:
            self.active_halt_request = {
                'timestamp_safeHaltReq': rospy.Time.now(),
                'timestamp_haltObserved': None,
                'timestamp_enforcedStop': None
            }

    def publish_event(self, state_name, event_name):
        global audit_event_pub
        if audit_event_pub is not None:
            audit_event_pub.publish(String(data="{}.{}".format(state_name, event_name)))

    def log_event(self, state_name, event_name):
        self.event_log.append({"time": rospy.Time.now().to_sec(), "state": state_name, "event": event_name})
        self.publish_event(state_name, event_name)

    def enter_state(self, state_name):
        self.leave_state()
        now = rospy.Time.now()
        self.current_state = state_name
        self.current_state_start = now

        if state_name == 'WaitForHalt':
            self.start_halt_request_timer()
            self.t5['WaitForHalt']['Total_number_of_times_WaitForHalt'] += 1
        elif state_name == 'SafeHaltActive':
            self.t6['SafeHaltActive']['Total_number_of_times_SafeHaltActive'] += 1

    def leave_state(self):
        if self.current_state is not None and self.current_state_start is not None:
            elapsed = (rospy.Time.now() - self.current_state_start).to_sec()
            if self.current_state in self.t4:
                self.t4[self.current_state]['time_spent_in_this_state'] += elapsed
            
            if self.current_state in ['WaitForHalt', 'SafeHaltActive'] and self.active_halt_request:
                self.m13_episodes.append(self.active_halt_request)
                self.active_halt_request = None
        self.current_state = None
        self.current_state_start = None

    def record_geofence_violation(self):
        if self.current_state in self.t4: self.t4[self.current_state]['count_geofenceViolation'] += 1

    def record_halt_observed(self):
        if self.active_halt_request and self.active_halt_request['timestamp_haltObserved'] is None:
            self.active_halt_request['timestamp_haltObserved'] = rospy.Time.now()
            self.t5['WaitForHalt']['count_haltObserved'] += 1

    def record_tick(self):
        if self.current_state == 'WaitForHalt': self.t5['WaitForHalt']['count_tick'] += 1

    def record_move(self):
        if self.current_state == 'SafeHaltActive': self.t6['SafeHaltActive']['count_move'] += 1

    def save_to_file(self, filepath, outcome):
        self.leave_state()
        if self.enforced_stop_triggered: outcome = "EnforcedStop"

        m13_processed = []
        for ep in self.m13_episodes:
            t_req = ep['timestamp_safeHaltReq'].to_sec()
            m13_val, mode = 0.0, "incomplete"
            if ep['timestamp_enforcedStop']:
                m13_val = ep['timestamp_enforcedStop'].to_sec() - t_req
                mode = "enforced"
            elif ep['timestamp_haltObserved']:
                m13_val = ep['timestamp_haltObserved'].to_sec() - t_req
                mode = "observed"
            m13_processed.append({"m13_response_time": round(m13_val, 4), "stop_mode": mode, "start_time": t_req})

        m14_latency = "N/A"
        if self.timestamp_geofence_violation and self.timestamp_enforced_stop:
            m14_latency = round((self.timestamp_enforced_stop - self.timestamp_geofence_violation).to_sec(), 4)

        data = {
            "mission_outcome": outcome,
            "M12_min_dist_observed": round(self.min_dist_observed, 4) if self.min_dist_observed != float('inf') else None,
            "M14_geofence_latency": m14_latency,
            "M13_Metrics": m13_processed,
            "event_log": self.event_log,
            "Table1": self.t1, "Table2": self.t2, "Table3": self.t3, 
            "Table4": self.t4, "Table5": self.t5, "Table6": self.t6
        }
        with open(filepath, 'a') as f:
            f.write(json.dumps(data) + "\n")

tracker = RunTracker()

_is_measured_leg = False
metrics_pub = None

NAV_PROFILE = {
    'planner_frequency':    1.0,
    'planner_patience':     5.0,
    'controller_patience':  15.0,
    'max_planning_retries': 3,
}

ABORT_PROFILE = {
    'planner_frequency':    1.0,
    'planner_patience':     5.0,
    'controller_patience':  15.0,
    'max_planning_retries': 3,
}

ENTRANCE_MB_PROFILE = {            
    'planner_frequency':    1.0,
    'planner_patience':     5.0,
    'controller_patience':  15.0,
    'max_planning_retries': 3,
}

DWA_ENTRANCE_PROFILE = {
    'max_vel_theta':      0.3,
    'path_distance_bias': 1.0,  
    'goal_distance_bias': 0.6,
    'occdist_scale':      0.1,  
}

DWA_NAV_PROFILE = {
    'max_vel_theta':      0.3,
    'path_distance_bias': 1.2,
    'goal_distance_bias': 0.5,
    'occdist_scale':      0.1,
}

DWA_ABORT_PROFILE = {
    'max_vel_theta':      0.3,
    'path_distance_bias': 1.2,
    'goal_distance_bias': 0.5,
    'occdist_scale':      0.1,
}

def set_dwa_profile(profile):
    try:
        client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS', timeout=3.0)
        client.update_configuration(profile)
        rospy.loginfo("DWA profile set: %s", profile)
    except Exception as e:
        rospy.logwarn("Could not set DWA profile: %s", e)

def set_move_base_profile(profile):
    try:
        client = dynamic_reconfigure.client.Client('move_base', timeout=3.0)
        client.update_configuration(profile)
        rospy.loginfo("move_base profile set: %s", profile)
    except Exception as e:
        rospy.logwarn("Could not set move_base profile: %s", e)

WAYPOINT_YAW = {}

ABORT_YAW = {
    'entrance':  math.pi / 2,   
    'exit':      math.pi / 2,   
    'refuge':    math.pi,        
}

tf_buffer = None
_nav_cancel_client = None
_discrepancy_confirmed = False

_cancel_pub = None
_cmd_pub = None
_cmd_filtered_pub = None

def get_robot_pose():
    try:
        trans = tf_buffer.lookup_transform('map', 'base_link',
                                           rospy.Time(0),
                                           rospy.Duration(1.0))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        return CPoint(x, y, z)
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("tf lookup failed: %s", e)
        return None

def stop_now():
    """Cancel any active move_base goal and pump zero velocity.

    Merged from smach_integrated_NH.py, with one addition for this launch
    topology. Two things it fixes versus publishing a single Twist from
    inside a callback:
      - the publishers are module-level and created in main(), so the
        connection is up before the first stop (a Twist published on a
        just-advertised publisher is normally dropped);
      - the zero is repeated (10 messages at 50 Hz, ~0.2 s) rather than
        sent once, so it survives move_base's own 20 Hz stream while the
        cancel is being processed.

    Zeros go to BOTH velocity topics. The launch files remap move_base
    /cmd_vel -> /cmd_vel_filtered, and the Gazebo skid-steer plugin
    (my_scout_mini_FIXED.xacro) plus scout_base.launch both subscribe to
    /cmd_vel_filtered. move_base therefore writes straight to the base
    input and never passes through cmd_vel_interceptor_node, which reads
    /cmd_vel. Publishing on /cmd_vel alone only reaches the base by way
    of the interceptor; /cmd_vel_filtered is the direct path.
    """
    if _cancel_pub is not None:
        _cancel_pub.publish(GoalID())

    if _cmd_pub is None and _cmd_filtered_pub is None:
        return

    stop_msg = Twist()
    rate = rospy.Rate(50)
    for _ in range(10):
        if rospy.is_shutdown():
            break
        if _cmd_pub is not None:
            _cmd_pub.publish(stop_msg)
        if _cmd_filtered_pub is not None:
            _cmd_filtered_pub.publish(stop_msg)
        rate.sleep()

def scan_cb(msg):
    tracker.record_scan(msg)

def odom_monitor_cb(msg):
    actual_lin = msg.twist.twist.linear.x
    actual_ang = msg.twist.twist.angular.z
    if abs(actual_lin) < 0.01 and abs(actual_ang) < 0.01:
        tracker.record_halt_observed()

def gwendolen_state_cb(msg):
    if msg.data in ['Monitoring', 'WaitForHalt', 'SafeHaltActive']:
        tracker.enter_state(msg.data)
    elif msg.data == 'EnforcedStop':
        tracker.record_enforced_stop_time()
        if tracker.active_halt_request:
            tracker.active_halt_request['timestamp_enforcedStop'] = rospy.Time.now()
        tracker.enforced_stop_triggered = True

def gwendolen_control_cb(msg):
    global _nav_cancel_client
    if msg.data:
        rospy.logerr("GWENDOLEN ENFORCED STOP RECEIVED! Cancelling navigation.")
        
        if _nav_cancel_client is not None:
            _nav_cancel_client.cancel_all_goals()

        stop_now()

def geofence_violation_cb(msg):
    if msg.data: tracker.record_geofence_violation()

def gwendolen_tick_cb(msg):
    if msg.data: tracker.record_tick()

def gwendolen_move_percept_cb(msg):
    if msg.data: tracker.record_move()

class InspectNext(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next_goal', 'mission_complete', 'aborted'],
                             input_keys=['mission_queue', 'previous_loc_name'],
                             output_keys=['current_goal_pose', 'current_loc_name', 'previous_loc_name'])

    def execute(self, userdata):
        if len(userdata.mission_queue) == 0:
            rospy.loginfo("All waypoints visited. Mission complete.")
            return 'mission_complete'

        loc_name = userdata.mission_queue.pop(0)

        global _is_measured_leg
        if loc_name != 'entrance':
            _is_measured_leg = True
            if metrics_pub:
                metrics_pub.publish(Bool(data=True))
        else:
            _is_measured_leg = False
            if metrics_pub:
                metrics_pub.publish(Bool(data=False))

        if _is_measured_leg:
            tracker.t1['Planning']['entered'] += 1
            tracker.log_event('Planning', 'entered')

        if loc_name not in LOCATIONS:
            rospy.logerr("Location '%s' not found in LOCATIONS dict", loc_name)
            if _is_measured_leg:
                tracker.t1['Planning']['plan_fail'] += 1
                tracker.log_event('Planning', 'plan_fail')
            return 'aborted'

        target = LOCATIONS[loc_name]
        rospy.loginfo("Next target: %s (%s)", loc_name, target)

        if loc_name in WAYPOINT_YAW:
            yaw = WAYPOINT_YAW[loc_name]
        elif len(userdata.mission_queue) > 0:
            next_name = userdata.mission_queue[0]
            if next_name in LOCATIONS:
                nxt = LOCATIONS[next_name]
                yaw = math.atan2(nxt.y - target.y, nxt.x - target.x)
            else:
                yaw = 0.0
        elif userdata.previous_loc_name and userdata.previous_loc_name in LOCATIONS:
            prev = LOCATIONS[userdata.previous_loc_name]
            yaw = math.atan2(target.y - prev.y, target.x - prev.x)
        else:
            yaw = 0.0

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target.x
        goal.target_pose.pose.position.y = target.y
        goal.target_pose.pose.position.z = target.z
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        userdata.current_goal_pose = goal.target_pose
        userdata.current_loc_name = loc_name
        userdata.previous_loc_name = loc_name

        if _is_measured_leg:
            tracker.t1['Planning']['plan_ok'] += 1
            tracker.log_event('Planning', 'plan_ok')
            tracker.t2['ExecConstraint']['entered'] += 1
            tracker.log_event('ExecConstraint', 'entered')

        return 'next_goal'

class InspectZone(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['inspect_ok'],
                             input_keys=['current_loc_name'])

    def execute(self, userdata):
        loc_name = userdata.current_loc_name
        rospy.loginfo("INSPECTION ongoing at %s", loc_name)

        global _is_measured_leg
        if _is_measured_leg:
            tracker.t2['ExecConstraint']['arrived'] += 1
            tracker.log_event('ExecConstraint', 'arrived')
            tracker.t3['Inspect']['entered'] += 1
            tracker.log_event('Inspect', 'entered')

        if loc_name in LOCATIONS:
            target_cpoint = LOCATIONS[loc_name]
            robot_pose = get_robot_pose()

            if robot_pose is not None:
                checker = LocationReached(target_cpoint, threshold=0.5)
                status = checker.update_location_status(robot_pose, rospy.Time.now())

                if status == LocationProps.REACHED:
                    rospy.loginfo("LocReached confirmed: %s (dist=%.2f)",
                                  loc_name, target_cpoint.distance(robot_pose))
                else:
                    rospy.logwarn("LocReached check: %s returned %s (dist=%.2f)",
                                  loc_name, status, target_cpoint.distance(robot_pose))
            else:
                rospy.logwarn("Could not get robot pose for LocReached check")

        rospy.sleep(2.0)
        rospy.loginfo("Inspection at %s finished.", loc_name)

        if _is_measured_leg:
            tracker.t3['Inspect']['inspect_ok'] += 1
            tracker.log_event('Inspect', 'inspect_ok')

        return 'inspect_ok'

class OnAbort(smach.State):
    def __init__(self, location_name, target_cpoint, preturn=False,
                 set_profile=True, mb_profile=None, dwa_profile=None):
        smach.State.__init__(self, outcomes=['reached', 'failed'])
        self.loc_name = location_name
        self.target_point = target_cpoint
        self.preturn = preturn
        self.set_profile = set_profile
        self.mb_profile = mb_profile if mb_profile is not None else ABORT_PROFILE
        self.dwa_profile = dwa_profile if dwa_profile is not None else DWA_ABORT_PROFILE
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def _turn_180(self):
        pose = get_robot_pose()
        if pose is not None:
            rospy.loginfo("ABORT: pose before on_abort: x=%.2f y=%.2f", pose.x, pose.y)
        else:
            rospy.logwarn("ABORT: could not get pose before on_abort")

        self.client.cancel_all_goals()
        rospy.sleep(0.3)

        turn_rate = 0.5   
        turn_duration = math.pi / turn_rate
        twist = Twist()
        twist.angular.z = turn_rate

        rate = rospy.Rate(10)
        start = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() < turn_duration:
            if rospy.is_shutdown():
                break
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.cmd_pub.publish(Twist())  
        rospy.sleep(0.3)
        rospy.loginfo("ABORT: on_abort, heading to %s", self.loc_name)

    def execute(self, userdata):
        rospy.logwarn("ABORT: Attempting to reach %s", self.loc_name)

        plan_state = "AbortPlan{}".format(self.loc_name.capitalize())
        
        if plan_state in tracker.t1:
            tracker.t1[plan_state]['entered'] += 1
            tracker.log_event(plan_state, 'entered')

        if self.preturn:
            self._turn_180()

        if self.set_profile:
            set_move_base_profile(self.mb_profile)
            set_dwa_profile(self.dwa_profile)

        rospy.sleep(0.5)

        if not self.client.wait_for_server(timeout=rospy.Duration(2.0)):
            rospy.logerr("ABORT: move_base unavailable")
            if plan_state in tracker.t1:
                tracker.t1[plan_state]['plan_fail'] += 1
                tracker.log_event(plan_state, 'plan_fail')
            return 'failed'

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.target_point.x
        goal.target_pose.pose.position.y = self.target_point.y
        goal.target_pose.pose.position.z = self.target_point.z
        yaw = ABORT_YAW.get(self.loc_name.lower(), 0.0)
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.client.send_goal(goal)

        if plan_state in tracker.t1:
            tracker.t1[plan_state]['plan_ok'] += 1
            tracker.log_event(plan_state, 'plan_ok')
        tracker.t2['AbortExecConstraint']['entered'] += 1
        tracker.log_event('AbortExecConstraint', 'entered')

        finished = self.client.wait_for_result(timeout=rospy.Duration(60.0))

        if finished and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("ABORT: move_base reports success for %s", self.loc_name)
            tracker.t2['AbortExecConstraint']['arrived'] += 1
            tracker.log_event('AbortExecConstraint', 'arrived')
            return 'reached'

        self.client.cancel_all_goals()
        robot_pose = get_robot_pose()
        if robot_pose is not None:
            dist = self.target_point.distance(robot_pose)
            rospy.loginfo("ABORT: move_base failed for %s, actual dist=%.2f",
                          self.loc_name, dist)
            if dist < 1.0:
                rospy.loginfo("ABORT: Close enough to %s, accepting", self.loc_name)
                tracker.t2['AbortExecConstraint']['arrived'] += 1
                tracker.log_event('AbortExecConstraint', 'arrived')
                return 'reached'

        rospy.logwarn("ABORT: Failed to reach %s. Falling through.", self.loc_name)
        tracker.t2['AbortExecConstraint']['stuck'] += 1
        tracker.log_event('AbortExecConstraint', 'stuck')
        return 'failed'

class SafeHalt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['halted'])
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.safehalt_pub = rospy.Publisher('/safehalt_request', Bool, queue_size=1, latch=True)

    def execute(self, userdata):
        rospy.logwarn("All abort targets unreachable. Engaging safe halt.")

        stop_msg = Twist()
        for _ in range(10):
            self.cmd_pub.publish(stop_msg)
            if _cmd_filtered_pub is not None:
                _cmd_filtered_pub.publish(stop_msg)
            rospy.sleep(0.1)

        tracker.start_halt_request_timer()
        self.safehalt_pub.publish(Bool(data=True))
        tracker.log_event('SAFE_HALT', 'entered')
        rospy.logwarn("Published /safehalt_request = True")

        rospy.logwarn("SAFE HALT ACTIVE - MANUAL INTERVENTION REQUIRED")
        return 'halted'

def discrepancy_cb(userdata, msg):
    global _discrepancy_confirmed
    if msg.data:
        if not _discrepancy_confirmed:
            _discrepancy_confirmed = True
            return True
        _discrepancy_confirmed = False
        rospy.logwarn("Discrepancy confirmed - cancelling move_base before abort.")
        
        global _is_measured_leg
        if _is_measured_leg:
            tracker.t2['ExecConstraint']['hazard'] += 1
            tracker.log_event('ExecConstraint', 'hazard')

        if _nav_cancel_client is not None:
            _nav_cancel_client.cancel_all_goals()
            rospy.sleep(0.25)   
        return False
    _discrepancy_confirmed = False
    return True

def build_abort_sm():
    sm_abort = smach.StateMachine(outcomes=['ABORTED', 'SYSTEM_HALTED'])

    with sm_abort:
        smach.StateMachine.add('TRY_ENTRANCE',
                               OnAbort('entrance', LOCATIONS['entrance'], preturn=True,
                                       mb_profile=ENTRANCE_MB_PROFILE, dwa_profile=DWA_ENTRANCE_PROFILE),
                               transitions={'reached': 'ABORTED',
                                            'failed':  'TRY_EXIT'})

        smach.StateMachine.add('TRY_EXIT',
                               OnAbort('exit', LOCATIONS['exit'], preturn=True),
                               transitions={'reached': 'ABORTED',
                                            'failed':  'TRY_REFUGE'})

        smach.StateMachine.add('TRY_REFUGE',
                               OnAbort('refuge', LOCATIONS['refuge'], preturn=True),
                               transitions={'reached': 'ABORTED',
                                            'failed':  'SAFE_HALT'})

        smach.StateMachine.add('SAFE_HALT', SafeHalt(),
                               transitions={'halted': 'SYSTEM_HALTED'})

    return sm_abort

def move_base_result_cb(userdata, status, result):
    global _is_measured_leg
    if _is_measured_leg and status in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
        
        if tracker.t1['Planning']['plan_ok'] > 0:
            tracker.t1['Planning']['plan_ok'] -= 1
            
        tracker.t1['Planning']['plan_fail'] += 1
        tracker.log_event('Planning', 'plan_fail')
        
        tracker.t2['ExecConstraint']['stuck'] += 1
        tracker.log_event('ExecConstraint', 'stuck')

def build_monitored_nav():
    nav_con = smach.Concurrence(
        outcomes=['nav_succeeded', 'nav_failed', 'discrepancy'],
        default_outcome='nav_failed',
        input_keys=['current_goal_pose'],
        child_termination_cb=lambda outcome_map: True,
        outcome_map={
            'nav_succeeded': {'NAVIGATE': 'succeeded'},
            'nav_failed':    {'NAVIGATE': 'aborted'},
            'discrepancy':   {'DISCREPANCY_WATCH': 'invalid'},
        })

    with nav_con:
        smach.Concurrence.add(
            'NAVIGATE',
            smach_ros.SimpleActionState('move_base', MoveBaseAction,
                                        goal_slots=['target_pose'],
                                        result_cb=move_base_result_cb),
            remapping={'target_pose': 'current_goal_pose'})

        smach.Concurrence.add(
            'DISCREPANCY_WATCH',
            smach_ros.MonitorState('/discrepancy_flag', Bool,
                                   discrepancy_cb))

    return nav_con

def main():
    rospy.init_node('smach_inspection_fsm')

    global audit_event_pub
    audit_event_pub = rospy.Publisher('/audit_events', String, queue_size=50)

    global _cancel_pub, _cmd_pub, _cmd_filtered_pub
    _cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    _cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    _cmd_filtered_pub = rospy.Publisher('/cmd_vel_filtered', Twist, queue_size=1)

    global metrics_pub
    metrics_pub = rospy.Publisher('/metrics_active', Bool, queue_size=1, latch=True)
    metrics_pub.publish(Bool(data=False))

    global tf_buffer
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.loginfo("Waiting for move_base action server...")
    actionlib.SimpleActionClient('move_base', MoveBaseAction).wait_for_server()
    rospy.loginfo("move_base connected. Starting state machine.")

    global _nav_cancel_client
    _nav_cancel_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    _nav_cancel_client.wait_for_server()
    rospy.sleep(0.5)

    rospy.Subscriber('/scan', LaserScan, scan_cb)

    rospy.Subscriber('/odom', Odometry, odom_monitor_cb)

    rospy.Subscriber('/gwendolen_geofence_violation', Bool, geofence_violation_cb)
    rospy.Subscriber('/gwendolen_state', String, gwendolen_state_cb)
    rospy.Subscriber('/gwendolen_tick', Bool, gwendolen_tick_cb)
    rospy.Subscriber('/gwendolen_move_percept', Bool, gwendolen_move_percept_cb)

    rospy.Subscriber('/gwendolen_control', Bool, gwendolen_control_cb)

    set_move_base_profile(NAV_PROFILE)
    set_dwa_profile(DWA_NAV_PROFILE)


    inspection_sequence = ['entrance', 'point1', 'point2', 'point3', 'point4']

    sm = smach.StateMachine(
        outcomes=['MISSION_SUCCESS', 'MISSION_FAILED',
                  'MISSION_ABORTED', 'SYSTEM_HALTED'])

    sm.userdata.mission_queue = inspection_sequence
    sm.userdata.current_goal_pose = None
    sm.userdata.current_loc_name = ""
    sm.userdata.previous_loc_name = ""

    with sm:
        smach.StateMachine.add(
            'INSPECT_NEXT', InspectNext(),
            transitions={'next_goal':        'MONITORED_NAV',
                         'mission_complete': 'MISSION_SUCCESS',
                         'aborted':          'MISSION_FAILED'})

        smach.StateMachine.add(
            'MONITORED_NAV', build_monitored_nav(),
            transitions={'nav_succeeded': 'INSPECT',
                         'nav_failed':    'ABORT_SM',
                         'discrepancy':   'ABORT_SM'})

        smach.StateMachine.add(
            'INSPECT', InspectZone(),
            transitions={'inspect_ok': 'INSPECT_NEXT'})

        smach.StateMachine.add(
            'ABORT_SM', build_abort_sm(),
            transitions={'ABORTED':       'MISSION_ABORTED',
                         'SYSTEM_HALTED': 'SYSTEM_HALTED'})

    sis = smach_ros.IntrospectionServer('inspection_server', sm, '/INSPECTION_SM')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("State machine finished: %s", outcome)

    save_path = rospy.get_param('~save_path', '/catkin_ws/safety_tables_data_physical.json')
    tracker.save_to_file(filepath=save_path, outcome=outcome)

    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass