#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from scipy.ndimage import distance_transform_edt
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2


class DiscrepancyMonitor:
    def __init__(self):
        rospy.init_node('discrepancy_monitor')

        # Beam endpoint is an outlier if distance to nearest known map obstacle
        # exceeds this value (metres). Tune up to reduce false positives.
        self.outlier_dist = rospy.get_param('~outlier_dist', 0.5)

        # Outlier beam hits per scan needed to start rising confidence.
        # A 0.3m cone at 1m distance subtends ~15-20 beams at 1-deg resolution.
        self.points_threshold = rospy.get_param('~points_threshold', 10) #10

        # Clip scan to this range window to ignore far-field noise.
        self.max_usable_range = rospy.get_param('~max_usable_range', 5.0)
        self.min_usable_range = rospy.get_param('~min_usable_range', 0.20)

        # Velocity gate: only check map-reality when robot is translating.
        # During pure rotation rf2o TF drifts, causing false outlier hits.
        self.min_linear_vel = rospy.get_param('~min_linear_vel', 0.05)   # m/s
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.3)  # rad/s
        self.robot_linear_vel = 0.0
        self.robot_angular_vel = 0.0

        # AMCL convergence gate.
        # Sum of var(x) + var(y) + var(yaw) from /amcl_pose covariance.
        # Discrepancy flag is suppressed until trace drops below this value.
        # At launch: trace ~0.32 (AMCL defaults). After convergence: <0.05.
        self.amcl_cov_threshold = rospy.get_param('~amcl_cov_threshold', 0.05)
        self.amcl_converged = False

        # Leaky integrator
        self.confidence = 0.0
        self.rise_rate = rospy.get_param('~rise_rate', 0.15)
        self.decay_rate = rospy.get_param('~decay_rate', 0.05)
        self.trigger_threshold = rospy.get_param('~trigger_threshold', 0.9)
        self.clear_threshold = rospy.get_param('~clear_threshold', 0.30)
        self.flag_active = False

        # Consecutive scan requirement: flag only triggers when this many
        # consecutive scans all have hits > points_threshold.
        # Resets to 0 on any scan with hits <= points_threshold.
        self.min_consecutive_scans = rospy.get_param('~min_consecutive_scans', 30)
        self.consecutive_count = 0

        # Cluster spread gate: only count a scan if the outlier beam endpoints
        # form a compact cluster (real object). Wall alignment artifacts produce
        # hundreds of hits spread over metres; a real cone/drum is ~0.1-0.2m.
        # Set to 0.0 to disable.
        self.max_cluster_spread = rospy.get_param('~max_cluster_spread', 0.4)

        # Location stability gate: consecutive centroids must stay within this
        # radius (metres) of each other to confirm it is the same obstacle.
        # A centroid jump > radius resets consecutive_count.
        self.location_radius = rospy.get_param('~location_radius', 0.3)
        self.prev_centroid = None   # (cx, cy) of last accepted compact cluster

        # State
        self.static_map = None
        # edt_map[y, x] = Euclidean distance (metres) from cell (x,y) to the
        # nearest occupied/unknown cell in the static map.
        # Beam endpoints with edt_map > outlier_dist hit something not in the map.
        self.edt_map = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.flag_pub = rospy.Publisher('/discrepancy_flag', Bool, queue_size=1)
        self.viz_pub = rospy.Publisher('/discrepancy_viz', PointCloud2, queue_size=1)

        rospy.Subscriber('/map_nav', OccupancyGrid,
                         self.static_map_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan,
                         self.scan_callback, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.amcl_pose_callback, queue_size=1)
        rospy.Subscriber('/odometry/filtered', Odometry,
                         self.odom_callback, queue_size=1)

        rospy.loginfo(
            "discrepancy_monitor: AMCL likelihood-field mode (scipy EDT), "
            "outlier_dist=%.2fm points_threshold=%d amcl_cov_threshold=%.3f",
            self.outlier_dist, self.points_threshold, self.amcl_cov_threshold)

    # ------------------------------------------------------------------
    # Odometry velocity gate
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        self.robot_linear_vel = abs(msg.twist.twist.linear.x)
        self.robot_angular_vel = abs(msg.twist.twist.angular.z)

    # ------------------------------------------------------------------
    # AMCL convergence gate
    # ------------------------------------------------------------------

    def amcl_pose_callback(self, msg):
        cov = msg.pose.covariance
        trace = cov[0] + cov[7] + cov[35]
        was_converged = self.amcl_converged
        self.amcl_converged = trace < self.amcl_cov_threshold
        if self.amcl_converged and not was_converged:
            rospy.loginfo(
                "discrepancy_monitor: AMCL converged (cov trace=%.4f < %.4f) - monitoring active",
                trace, self.amcl_cov_threshold)
        elif not self.amcl_converged and was_converged:
            rospy.logwarn(
                "discrepancy_monitor: AMCL diverged (cov trace=%.4f) - monitoring suspended",
                trace)

    # ------------------------------------------------------------------
    # Static map - build Euclidean distance transform
    # ------------------------------------------------------------------

    def static_map_callback(self, msg):
        self.static_map = msg

        raw = np.array(msg.data, dtype=np.int8).view(np.uint8).reshape(
            msg.info.height, msg.info.width).astype(np.int32)

        # map_server: 0=free, 100=occupied, -1=unknown (255 after uint8 cast).
        # Treat occupied AND unknown as obstacles.
        obstacle_mask = (raw > 50)

        # EDT: for every cell, distance in pixels to the nearest obstacle cell.
        # distance_transform_edt(~mask) gives distance FROM free cells TO nearest
        # True cell in obstacle_mask. Obstacle cells themselves get distance 0.
        dist_pixels = distance_transform_edt(~obstacle_mask)

        # Convert to metres
        self.edt_map = dist_pixels * msg.info.resolution

        rospy.loginfo(
            "discrepancy_monitor: EDT built (map %dx%d cells, res=%.3fm)",
            msg.info.width, msg.info.height, msg.info.resolution)

    # ------------------------------------------------------------------
    # Scan callback - AMCL likelihood field check
    # ------------------------------------------------------------------

    def scan_callback(self, scan):
        if self.static_map is None or self.edt_map is None:
            return
        if not self.amcl_converged:
            rospy.loginfo_throttle(5, "discrepancy_monitor: waiting for AMCL to converge...")
            self.confidence = 0.0
            self.consecutive_count = 0
            self.prev_centroid = None
            self.flag_pub.publish(Bool(data=self.flag_active))
            return
        if (self.robot_linear_vel < self.min_linear_vel or
                self.robot_angular_vel > self.max_angular_vel):
            rospy.loginfo_throttle(5,
                "discrepancy_monitor: suppressed (lin=%.2f ang=%.2f)",
                self.robot_linear_vel, self.robot_angular_vel)
            self.confidence = 0.0
            self.consecutive_count = 0
            self.prev_centroid = None
            self.flag_pub.publish(Bool(data=self.flag_active))
            return

        # TF: laser frame -> map frame
        try:
            trans = self.tf_buffer.lookup_transform(
                self.static_map.header.frame_id,
                scan.header.frame_id,
                scan.header.stamp,
                rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, "discrepancy_monitor: TF not ready: %s",
                                   type(e).__name__)
            return
        except Exception as e:
            rospy.logerr_throttle(10, "discrepancy_monitor: TF error: %s", e)
            return

        t_x = trans.transform.translation.x
        t_y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Valid range filter
        ranges = np.array(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        r_min = max(scan.range_min, self.min_usable_range)
        r_max = min(scan.range_max, self.max_usable_range)
        valid = (ranges > r_min) & (ranges < r_max) & np.isfinite(ranges)

        if not valid.any():
            self._update_confidence(0)
            self.flag_pub.publish(Bool(data=self.flag_active))
            return

        r_v = ranges[valid]
        a_v = angles[valid]

        # Beam endpoints in map frame (vectorized)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        lx = r_v * np.cos(a_v)
        ly = r_v * np.sin(a_v)
        mx = lx * cos_yaw - ly * sin_yaw + t_x
        my = lx * sin_yaw + ly * cos_yaw + t_y

        # Map grid indices of beam endpoints
        res = self.static_map.info.resolution
        ox  = self.static_map.info.origin.position.x
        oy  = self.static_map.info.origin.position.y
        w   = self.static_map.info.width
        h   = self.static_map.info.height

        gx = ((mx - ox) / res).astype(np.int32)
        gy = ((my - oy) / res).astype(np.int32)

        in_bounds = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        gx_b = gx[in_bounds]
        gy_b = gy[in_bounds]
        mx_b = mx[in_bounds]
        my_b = my[in_bounds]

        if len(gx_b) == 0:
            self._update_confidence(0)
            self.flag_pub.publish(Bool(data=self.flag_active))
            return

        # AMCL likelihood field:
        # Outlier = beam endpoint whose EDT distance exceeds outlier_dist.
        # These beams hit something that is NOT in the static map.
        dist_to_wall = self.edt_map[gy_b, gx_b]
        outlier_mask = dist_to_wall > self.outlier_dist
        valid_hits = int(outlier_mask.sum())

        # Cluster spread gate: reject scans where outlier points are spread over
        # a large area (wall alignment artifact) vs a compact object (real obstacle).
        # Mean distance of each outlier point from the cluster centroid.
        compact = True
        if valid_hits > 0 and self.max_cluster_spread > 0.0:
            bx = mx_b[outlier_mask]
            by = my_b[outlier_mask]
            cx = float(np.mean(bx))
            cy = float(np.mean(by))
            spread = float(np.mean(np.sqrt((bx - cx) ** 2 + (by - cy) ** 2)))
            compact = spread < self.max_cluster_spread
            rospy.loginfo_throttle(1,
                "discrepancy_monitor: outlier cluster centroid (%.2f, %.2f) "
                "spread=%.2fm hits=%d compact=%s",
                cx, cy, spread, valid_hits, compact)
        elif valid_hits > 0:
            bx = mx_b[outlier_mask]
            by = my_b[outlier_mask]
            cx = float(np.mean(bx))
            cy = float(np.mean(by))
            rospy.loginfo_throttle(1,
                "discrepancy_monitor: outlier cluster centroid (%.2f, %.2f) "
                "hits=%d", cx, cy, valid_hits)

        # Location stability gate: centroid must stay within location_radius of
        # the previous accepted centroid, confirming the same obstacle.
        location_stable = False
        if compact and valid_hits > 0:
            if self.prev_centroid is None:
                # First detection - accept and anchor
                self.prev_centroid = (cx, cy)
                location_stable = True
            else:
                dist_to_prev = np.sqrt((cx - self.prev_centroid[0]) ** 2 +
                                       (cy - self.prev_centroid[1]) ** 2)
                if dist_to_prev <= self.location_radius:
                    location_stable = True
                    # Smoothly update anchor toward new centroid
                    self.prev_centroid = (cx, cy)
                else:
                    rospy.loginfo_throttle(1,
                        "discrepancy_monitor: centroid jumped %.2fm > %.2fm - "
                        "resetting consecutive count",
                        dist_to_prev, self.location_radius)
                    self.prev_centroid = (cx, cy)
        elif not compact:
            self.prev_centroid = None

        # Only feed the leaky integrator if cluster is compact AND location stable
        self._update_confidence(valid_hits if location_stable else 0)
        self.flag_pub.publish(Bool(data=self.flag_active))

        if location_stable:
            pts3d = np.column_stack((bx, by, np.full(valid_hits, 0.5)))
            hdr = scan.header
            hdr.frame_id = self.static_map.header.frame_id
            self.viz_pub.publish(pc2.create_cloud_xyz32(hdr, pts3d.tolist()))

        if self.flag_active:
            rospy.logwarn_throttle(2,
                "discrepancy_monitor: CONFIRMED - map-reality discrepancy detected. Triggering Abort.")

    # ------------------------------------------------------------------
    # Leaky integrator
    # ------------------------------------------------------------------

    def _update_confidence(self, valid_hits):
        if valid_hits > self.points_threshold:
            self.consecutive_count += 1
            ratio = min(valid_hits / float(self.points_threshold), 3.0)
            self.confidence += self.rise_rate * ratio
        else:
            self.consecutive_count = 0
            self.confidence -= self.decay_rate

        self.confidence = max(0.0, min(1.0, self.confidence))

        if (not self.flag_active
                and self.confidence >= self.trigger_threshold
                and self.consecutive_count >= self.min_consecutive_scans):
            self.flag_active = True
            rospy.loginfo(
                "discrepancy_monitor: flag TRIGGERED "
                "(confidence=%.2f consecutive=%d)",
                self.confidence, self.consecutive_count)
        elif self.flag_active and self.confidence <= self.clear_threshold:
            self.flag_active = False
            self.consecutive_count = 0
            self.prev_centroid = None
            rospy.loginfo("discrepancy_monitor: flag CLEARED (confidence %.2f)",
                          self.confidence)

        rospy.loginfo_throttle(1,
            "discrepancy_monitor: outlier_hits=%d consecutive=%d confidence=%.2f flag=%s",
            valid_hits, self.consecutive_count, self.confidence, self.flag_active)


if __name__ == '__main__':
    monitor = DiscrepancyMonitor()
    rospy.spin()
