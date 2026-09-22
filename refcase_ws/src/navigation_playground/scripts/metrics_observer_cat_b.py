#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import os
import datetime
import math
import tf2_ros
from collections import OrderedDict
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# Import the AgileX Scout hardware status message
try:
    from scout_msgs.msg import ScoutStatus
except ImportError:
    rospy.logwarn("scout_msgs not found. Make sure the scout base packages are sourced.")

class CategoryBObserver:
    def __init__(self):
        rospy.init_node('metrics_observer_cat_b')

        self.test_name = rospy.get_param('~test_name', 'full_safety_audit_hardware')
        self.save_dir = rospy.get_param('~save_dir', '/catkin_ws/CategoryB_Logs')
        
        if not os.path.exists(self.save_dir): 
            os.makedirs(self.save_dir)
        self.save_path = os.path.join(self.save_dir, "result_{}.json".format(self.test_name))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Exact Physical Map Restricted Areas
        self.AREAS = [
            {'name': 'area_1', 'corners': [(-0.45, -0.71), (2.56, -0.71), (2.56, -1.30), (-0.45, -1.30)]},
            {'name': 'area_2', 'corners': [(0.56, -3.94), (1.60, -3.94), (1.60, -5.0), (0.56, -5.0)]},
            {'name': 'area_3', 'corners': [(3.27, -3.10), (4.13, -3.10), (4.13, -4.42), (3.27, -4.42)]},
            {'name': 'area_4', 'corners': [(5.80, 0.8), (8.06, 0.8), (8.06, -2.45), (5.80, -2.45)]}
        ]

        self.state_history = []
        self.hazard_active = False
        
        self.m12_min_dist = float('inf')
        self.t_geofence_violation = None
        self.t_enforced_stop = None
        self.m14_latency = None

        self.safe_halt_active = False
        self.t_safe_halt_req = None
        self.t_halt_observed = None
        self.m13_response_time = None
        
        # M5 Persistence variables
        self.command_motion_after_halt = False
        self.physical_motion_after_halt = False

        # Binary Flags
        self.m1_compliant = "YES"
        self.m2_compliant = "NO"
        self.m3_compliant = "YES"
        self.m4_compliant = "NO"
        self.m5_compliant = "NO"

        # --- Subscribers ---
        self.sm_sub = rospy.Subscriber('/inspection_server/smach/container_status', SmachContainerStatus, self.smach_cb)
        self.haz_sub = rospy.Subscriber('/discrepancy_flag', Bool, self.hazard_cb)
        self.halt_sub = rospy.Subscriber('/safehalt_request', Bool, self.halt_cb)
        
        # Monitor Software Intent (M5 commands)
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_cb)
        
        # Monitor Hardware Reality (M13 latency and M5 physical persistence)
        self.scout_sub = rospy.Subscriber('/scout_status', ScoutStatus, self.scout_status_cb)

        rospy.Timer(rospy.Duration(0.1), self.track_geofence)
        rospy.on_shutdown(self.save_results)

    def track_geofence(self, event):
        if not self.state_history or self.state_history[-1] != 'NAVIGATE':
            return
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
            rx, ry = trans.transform.translation.x, trans.transform.translation.y
            
            for area in self.AREAS:
                x_coords = [p[0] for p in area['corners']]
                y_coords = [p[1] for p in area['corners']]
                x_min, x_max = min(x_coords), max(x_coords)
                y_min, y_max = min(y_coords), max(y_coords)

                dx = max(x_min - rx, 0, rx - x_max)
                dy = max(y_min - ry, 0, ry - y_max)
                dist = math.sqrt(dx*dx + dy*dy)
                
                if dist < self.m12_min_dist:
                    self.m12_min_dist = dist
                
                if dist <= 0.0 and self.t_geofence_violation is None:
                    self.t_geofence_violation = rospy.Time.now().to_sec()
        except Exception:
            pass

    def hazard_cb(self, msg):
        if msg.data: self.hazard_active = True

    def halt_cb(self, msg):
        if msg.data and not self.safe_halt_active: 
            self.safe_halt_active = True
            # M13 Start Trigger
            self.t_safe_halt_req = rospy.Time.now().to_sec()

    def vel_cb(self, msg):
        """M5 LOGIC: Ensures the software stops sending commands."""
        if self.safe_halt_active:
            is_stopped = abs(msg.linear.x) <= 0.001 and abs(msg.angular.z) <= 0.001
            if not is_stopped:
                self.command_motion_after_halt = True

    def scout_status_cb(self, msg):
        """M13 & M5 LOGIC: Listens to the hardware base directly."""
        if self.safe_halt_active:
            # Scout hardware telemetry for linear and angular speed
            is_physically_stopped = abs(msg.linear_velocity) <= 0.001 and abs(msg.angular_velocity) <= 0.001
            
            # M13 Stop Trigger: The exact moment the wheels stop moving
            if is_physically_stopped and self.t_halt_observed is None and self.t_safe_halt_req is not None:
                self.t_halt_observed = rospy.Time.now().to_sec()
                self.m13_response_time = self.t_halt_observed - self.t_safe_halt_req

            # M5 Hardware Persistence: If the wheels turn again after stopping
            if not is_physically_stopped and self.t_halt_observed is not None:
                self.physical_motion_after_halt = True

    def smach_cb(self, msg):
        if not msg.active_states: return
        current_state = msg.active_states[-1] 
        
        if current_state in ['None', 'MONITORED_NAV', 'ABORT_SM', 'DISCREPANCY_WATCH']: return

        if not self.state_history or current_state != self.state_history[-1]:
            if current_state == 'NAVIGATE' and self.state_history:
                if self.state_history[-1] != 'INSPECT_NEXT': self.m1_compliant = "NO"
            if current_state == 'TRY_ENTRANCE' and self.hazard_active: 
                self.m2_compliant = "YES"
            recovery = ['TRY_ENTRANCE', 'TRY_EXIT', 'TRY_REFUGE', 'SAFE_HALT']
            trace = [s for s in self.state_history + [current_state] if s in recovery]
            if len(trace) > 1:
                l, p = trace[-1], trace[-2]
                if (l=='TRY_EXIT' and p!='TRY_ENTRANCE') or (l=='TRY_REFUGE' and p!='TRY_EXIT') or (l=='SAFE_HALT' and p!='TRY_REFUGE'):
                    self.m3_compliant = "NO"
            
            if current_state == 'SAFE_HALT':
                if self.t_geofence_violation is not None and self.t_enforced_stop is None:
                    self.t_enforced_stop = rospy.Time.now().to_sec()
                    self.m14_latency = self.t_enforced_stop - self.t_geofence_violation

            self.state_history.append(current_state)

    def save_results(self):
        # M4: Did it reach SAFE_HALT and fire the request?
        if self.safe_halt_active and 'SAFE_HALT' in self.state_history:
            self.m4_compliant = "YES"

        # M5: Requires BOTH no software commands AND no hardware movement after halt
        if self.m4_compliant == "YES" and not self.command_motion_after_halt and not self.physical_motion_after_halt:
            self.m5_compliant = "YES"

        report = OrderedDict()
        report["metadata"] = {"test_case": self.test_name, "timestamp": str(datetime.datetime.now())}
        
        m13_val = round(self.m13_response_time, 4) if self.m13_response_time is not None else "N/A"
        
        if self.t_geofence_violation is None:
            m14_val = "No Violation"
        elif self.t_geofence_violation is not None and self.t_enforced_stop is None:
            m14_val = "Breach without Safe Halt"
        else:
            m14_val = round(self.m14_latency, 4)

        results = OrderedDict()
        results["M1_Planning_Ordering_Compliant"] = self.m1_compliant
        results["M2_Abort_On_Hazard_Compliant"] = self.m2_compliant
        results["M3_Deterministic_Priority_Compliant"] = self.m3_compliant
        results["M4_Safe_Halt_Transition_Compliant"] = self.m4_compliant
        results["M5_Safe_Halt_Persistence_Compliant"] = self.m5_compliant
        results["M12_Min_Distance_to_Boundary_m"] = round(self.m12_min_dist, 3) if self.m12_min_dist != float('inf') else "N/A"
        results["M13_Safe_Halt_Response_Time_s"] = m13_val
        results["M14_Geofence_Enforcement_Latency_s"] = m14_val
        results["Trace"] = self.state_history

        report["Table_3_Audit_Results"] = results

        try:
            with open(self.save_path, 'w') as f:
                json.dump(report, f, indent=4)
            rospy.loginfo("Hardware-Level Audit Complete. Saved to: %s", self.save_path)
        except Exception as e:
            rospy.logerr("Failed to save JSON: %s", str(e))

if __name__ == '__main__':
    try:
        CategoryBObserver()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass