#!/usr/bin/env python

import rospy
import json
import os
from sensor_msgs.msg import LaserScan

class MetricsObserver:
    def __init__(self, run_id="run_default"):
        self.run_id = run_id
        self.save_path = rospy.get_param('~save_path', "/catkin_ws/mission_results_A.json")
        
        # Defines the distance from the center-mounted LiDAR to the outer bumper.
        self.robot_radius = rospy.get_param('~robot_radius', 0.25) 

        # SRAS safety threshold (0.50m from bumper)
        self.threshold_sras = 0.5
        
        # SS safety threshold (0.2m from bumper)
        self.threshold_ss   = 0.2  
        
        # Initialize min distance to infinity to ensure any valid reading is captured
        self.min_dist_observed = float('inf')

        # Subscriber for LiDAR scan
        # We treat every point as a potential obstacle or restricted boundary 
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        
        rospy.loginfo("[Observer] Monitoring M11 & M12. Limits: SRAS=0.50m, SS=0.2m")
        rospy.loginfo("[Observer] Accounting for Robot Radius: %.2fm", self.robot_radius)

    def scan_cb(self, msg):
        """
        Processes LiDAR scan to track the minimum distance observed.
        Treats all physical returns as either an obstacle or boundary (M12).
        Accounts for the robot's physical body radius.
        """
        valid_distances = []
        
        # Filter for valid ranges and adjust for the robot's physical footprint
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Subtract the robot's radius to get the distance from the bumper.
                # max(0.0, ...) ensures we don't get negative distances if something 
                # overhangs the sensor slightly.
                true_dist = max(0.0, r - self.robot_radius)
                valid_distances.append(true_dist)
        
        if valid_distances:
            current_min = min(valid_distances)
            
            # Update the 'worst-case' minimum distance for the current run
            if current_min < self.min_dist_observed:
                self.min_dist_observed = current_min

    def shutdown_hook(self):
        """
        Called when the ROS node is killed at the end of a run.
        Calculates violation flags and saves the final metrics to JSON.
        """
        # M11 Logic: Check if distance fell below the two specific thresholds
        violation_sras = self.min_dist_observed < self.threshold_sras
        violation_ss   = self.min_dist_observed < self.threshold_ss
        
        # Prepare the physical metrics data packet
        # M12 definition: min(distance to restricted boundary)
        data = {
            "run_id": self.run_id,
            "M11_SRAS_violation": bool(violation_sras),
            "M11_SS_violation": bool(violation_ss),
            "M12_min_dist": round(self.min_dist_observed, 4) if self.min_dist_observed != float('inf') else None
        }
        
        try:
            # Append as a single line of JSON (Standard for audit processing)
            with open(self.save_path, "a") as f:
                f.write(json.dumps(data) + "\n")
            rospy.loginfo("[Observer] Run %s Data Saved: MinDist=%.3f", self.run_id, self.min_dist_observed)
        except Exception as e:
            rospy.logerr("[Observer] Failed to save JSON metrics: %s", e)

if __name__ == '__main__':
    rospy.init_node('metrics_observer')
    
    # Retrieve the run_id from launch file or command line
    run_id_param = rospy.get_param('~run_id', 'run_default')
    
    obs = MetricsObserver(run_id=run_id_param)
    
    # Ensure data is saved even if the node is terminated abruptly
    rospy.on_shutdown(obs.shutdown_hook)
    
    rospy.spin()