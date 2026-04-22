#!/usr/bin/env python3
"""
Minimal ROS2 node for fake semantic navigation.
Subscribes to a text command (e.g. "go to table"), parses the object label,
looks up a semantic region from YAML (center, radius, yaw, per-object arrival thresholds),
and sends NavigateToPose to an approach point outside the cylinder: along center→robot at
distance (radius + semantic_nav_goal_standoff_m), never the raw center.

Optional map gating: only allow navigation when the goal cell is explored (known).

Fake object discovery (no vision): each object is a disk in the map; when enough cells inside
the disk are occupied in SLAM /map, the object is marked discovered and markers are published.

Region-based arrival (use_costmap_proximity_arrival): cancel Nav2 when the robot is next to the
object cylinder — surface clearance = distance(robot, center) - radius — is within
arrival_distance_tolerance and global costmap at the robot is >= arrival_cost_threshold —
then rotate in place to the YAML yaw via cmd_vel.
"""

import os
import sys
import math
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RcDuration
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Duration as BuiltinDuration
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# Allow importing command_parser and map_gating when run via ros2 run (same install dir).
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
import command_parser  # noqa: E402
import map_gating  # noqa: E402


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw (radians) to quaternion (x, y, z, w). Rotation around z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_transform(transform) -> float:
    q = transform.transform.rotation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FakeSemanticNavigationNode(Node):
    def __init__(self):
        super().__init__("fake_semantic_navigation_node")

        self.declare_parameter("user_command_topic", "user_command")
        self.declare_parameter("semantic_cmd_result_topic", "semantic_cmd_result")
        self.declare_parameter("semantic_goal_pose_topic", "semantic_goal_pose")
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("semantic_goals_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("use_map_gating", True)
        # Fake recognition: SLAM /map obstacle inside cylinder disk → discovered → RViz markers.
        self.declare_parameter("use_fake_object_discovery", True)
        self.declare_parameter("semantic_object_radius_m", 0.25)
        self.declare_parameter("fake_discovery_occupied_threshold", 50)
        self.declare_parameter("fake_discovery_min_fraction", 0.12)
        self.declare_parameter("fake_discovery_min_cells", 2)
        self.declare_parameter("semantic_markers_topic", "semantic_object_markers")

        # Proximity arrival: global costmap (same grid type as /map; cost 0–255).
        self.declare_parameter("use_costmap_proximity_arrival", True)
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("semantic_arrival_cost_threshold", 75)
        self.declare_parameter("semantic_arrival_xy_tolerance_m", 0.7)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("proximity_check_rate_hz", 10.0)
        self.declare_parameter("rotate_yaw_tolerance_rad", 0.12)
        self.declare_parameter("rotate_max_angular_speed", 1.2)
        self.declare_parameter("rotate_kp", 2.0)
        # Nav2 pose XY: on the ray from object center toward the robot, at (radius + standoff)
        # from center (free-space ring outside the semantic cylinder). Extra clearance beyond r.
        self.declare_parameter("semantic_nav_goal_standoff_m", 0.15)
        # TF: use latest transform (time=0) in this node's clock domain; avoids extrapolation errors
        # when requesting "now" vs buffered stamp. Retries cover brief startup delay before /tf arrives.
        self.declare_parameter("tf_lookup_timeout_sec", 1.0)
        self.declare_parameter("tf_lookup_max_attempts", 5)
        self.declare_parameter("tf_lookup_retry_delay_sec", 0.05)

        self._goals, self._allowed_objects, self._frame_id = self._load_semantic_goals()
        self._goals = {label: self._coerce_semantic_goal(raw) for label, raw in self._goals.items()}

        if not self._goals:
            self.get_logger().error("No semantic goals loaded. Check semantic_goals_file.")
            return

        nav_action = self.get_parameter("navigate_to_pose_action").get_parameter_value().string_value
        cmd_topic = self.get_parameter("user_command_topic").get_parameter_value().string_value
        result_topic = self.get_parameter("semantic_cmd_result_topic").get_parameter_value().string_value
        goal_pose_topic = self.get_parameter("semantic_goal_pose_topic").get_parameter_value().string_value
        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self._use_map_gating = self.get_parameter("use_map_gating").get_parameter_value().bool_value
        self._use_fake_discovery = self.get_parameter("use_fake_object_discovery").get_parameter_value().bool_value
        markers_topic = self.get_parameter("semantic_markers_topic").get_parameter_value().string_value
        self._use_proximity = (
            self.get_parameter("use_costmap_proximity_arrival").get_parameter_value().bool_value
        )
        self._costmap_topic = self.get_parameter("costmap_topic").get_parameter_value().string_value
        self._robot_base = self.get_parameter("robot_base_frame").get_parameter_value().string_value
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value

        self._latest_map = None
        self._latest_costmap = None
        self._object_order = sorted(self._goals.keys())
        self._discovered_objects: set[str] = set()

        self._action_client = ActionClient(self, NavigateToPose, nav_action)
        self._sub = self.create_subscription(String, cmd_topic, self._command_callback, 10)
        self._result_pub = self.create_publisher(String, result_topic, 10)
        self._goal_pose_pub = self.create_publisher(PoseStamped, goal_pose_topic, 10)
        self._marker_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        if self._use_map_gating or self._use_fake_discovery:
            self._map_sub = self.create_subscription(
                OccupancyGrid, map_topic, self._map_callback, 10
            )

        if self._use_proximity:
            self._costmap_sub = self.create_subscription(
                OccupancyGrid, self._costmap_topic, self._costmap_callback, 10
            )
            rate = self.get_parameter("proximity_check_rate_hz").get_parameter_value().double_value
            if rate <= 0.0:
                rate = 10.0
            self._tick_timer = self.create_timer(1.0 / rate, self._mission_tick)
        else:
            self._tick_timer = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # idle | navigate | rotate
        self._phase = "idle"
        self._nav_goal_handle = None
        self._proximity_cancel = False
        # Region mission: center, radius, yaw, per-object thresholds (dict) while active
        self._mission_pose = None

        self.get_logger().info(
            "fake_semantic_navigation_node started. Objects: %s. Map gating: %s. Fake discovery: %s. "
            "Markers: %s. Proximity arrival: %s."
            % (
                ", ".join(sorted(self._allowed_objects)),
                self._use_map_gating,
                self._use_fake_discovery,
                markers_topic,
                self._use_proximity,
            )
        )

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        self._latest_costmap = msg

    def _abort_mission_publish(self, text: str) -> None:
        self._phase = "idle"
        self._mission_pose = None
        self._nav_goal_handle = None
        self._proximity_cancel = False
        self._publish_twist_zero()
        self._publish_result(text)

    def _publish_twist_zero(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _get_robot_xy_yaw(self):
        """
        Return (x, y, yaw) of robot base in self._frame_id (usually map), or None.

        lookup_transform(target=self._frame_id, source=self._robot_base): pose of base in map.
        Uses latest available TF (time zero in this node's clock type), not get_clock().now(),
        so sim-time and TF stamp skew do not cause false failures. Retries if TF not ready yet.
        """
        # Latest transform: time zero in this node's clock (matches tf2 / use_sim_time).
        clock_type = self.get_clock().clock_type
        when = Time(seconds=0, nanoseconds=0, clock_type=clock_type)
        timeout_sec = float(self.get_parameter("tf_lookup_timeout_sec").get_parameter_value().double_value)
        max_attempts = max(1, int(self.get_parameter("tf_lookup_max_attempts").get_parameter_value().integer_value))
        retry_delay = float(self.get_parameter("tf_lookup_retry_delay_sec").get_parameter_value().double_value)
        timeout = RcDuration(seconds=max(0.01, timeout_sec))

        last_err = None
        for attempt in range(max_attempts):
            try:
                t = self._tf_buffer.lookup_transform(
                    self._frame_id,
                    self._robot_base,
                    when,
                    timeout=timeout,
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                yaw = yaw_from_transform(t)
                if attempt > 0:
                    self.get_logger().info(
                        "TF ok: %s pose in '%s' (succeeded on attempt %d/%d; latest transform)."
                        % (self._robot_base, self._frame_id, attempt + 1, max_attempts)
                    )
                return (x, y, yaw)
            except Exception as e:
                last_err = e
                self.get_logger().debug(
                    "TF lookup '%s'<-'%s' attempt %d/%d failed: %s"
                    % (self._frame_id, self._robot_base, attempt + 1, max_attempts, e)
                )
                if attempt < max_attempts - 1:
                    time.sleep(retry_delay)
                else:
                    self.get_logger().warning(
                        "TF failed: '%s'<-'%s' after %d attempts (latest, clock=%s): %s"
                        % (
                            self._frame_id,
                            self._robot_base,
                            max_attempts,
                            clock_type,
                            last_err,
                        )
                    )
        return None

    def _mission_tick(self) -> None:
        """Proximity check during navigate; in-place rotation during rotate."""
        if not self._use_proximity:
            return

        if self._phase == "navigate" and self._mission_pose is not None:
            m = self._mission_pose
            cx, cy = m["cx"], m["cy"]
            obj_r = m["radius"]
            target_yaw = m["target_yaw"]
            pose = self._get_robot_xy_yaw()
            if pose is None:
                return
            rx, ry, _ryaw = pose
            dist_center = math.hypot(rx - cx, ry - cy)
            # Signed distance to outer surface along radial line (negative = inside cylinder).
            surface_clearance = dist_center - obj_r
            thresh = int(m["cost_thresh"])
            surface_tol = float(m["surface_tol"])

            cost = None
            if self._latest_costmap is not None:
                cost = map_gating.sample_occupancy_cost_at_world(self._latest_costmap, rx, ry)

            if cost is not None and cost >= thresh and surface_clearance <= surface_tol:
                self.get_logger().info(
                    "Region arrival: cost=%d (>=%d), surface_clearance=%.3f (<= %.3f), "
                    "dist_center=%.3f, r=%.3f — cancel Nav2, align yaw."
                    % (cost, thresh, surface_clearance, surface_tol, dist_center, obj_r)
                )
                self._phase = "rotate"
                self._proximity_cancel = True
                if self._nav_goal_handle is not None:
                    self._nav_goal_handle.cancel_goal_async()

        elif self._phase == "rotate" and self._mission_pose is not None:
            target_yaw = self._mission_pose["target_yaw"]
            pose = self._get_robot_xy_yaw()
            if pose is None:
                return
            _rx, _ry, ryaw = pose
            yaw_err = normalize_angle(target_yaw - ryaw)
            yaw_tol = (
                self.get_parameter("rotate_yaw_tolerance_rad").get_parameter_value().double_value
            )
            wmax = self.get_parameter("rotate_max_angular_speed").get_parameter_value().double_value
            kp = self.get_parameter("rotate_kp").get_parameter_value().double_value

            if abs(yaw_err) < yaw_tol:
                self._publish_twist_zero()
                self._phase = "idle"
                self._mission_pose = None
                self._nav_goal_handle = None
                self.get_logger().info("Heading aligned.")
                self._publish_result("Navigation finished.")
                return

            w = max(-wmax, min(wmax, kp * yaw_err))
            tw = Twist()
            tw.angular.z = w
            self._cmd_vel_pub.publish(tw)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        self._update_fake_object_discovery()
        self._publish_semantic_markers()

    def _update_fake_object_discovery(self) -> None:
        """Mark predefined objects discovered when SLAM map shows obstacle inside cylinder footprint (sticky)."""
        if not self._use_fake_discovery or self._latest_map is None:
            return
        occ_thr = int(
            self.get_parameter("fake_discovery_occupied_threshold").get_parameter_value().integer_value
        )
        frac = float(self.get_parameter("fake_discovery_min_fraction").get_parameter_value().double_value)
        min_cells = int(self.get_parameter("fake_discovery_min_cells").get_parameter_value().integer_value)
        for label in self._object_order:
            if label in self._discovered_objects:
                continue
            pose_data = self._goals[label]
            cx = float(pose_data["x"])
            cy = float(pose_data["y"])
            r = float(pose_data["radius"])
            if map_gating.is_semantic_disk_occupied_in_slam_map(
                self._latest_map,
                cx,
                cy,
                r,
                occupied_threshold=occ_thr,
                min_occupied_fraction=frac,
                min_occupied_cells=min_cells,
            ):
                self._discovered_objects.add(label)
                self.get_logger().info(
                    "Fake object discovery: '%s' at (%.2f, %.2f), disk r=%.2f m — publishing semantic marker."
                    % (label, cx, cy, r)
                )

    def _publish_semantic_markers(self) -> None:
        if self._latest_map is None:
            return
        stamp = self.get_clock().now().to_msg()
        n = len(self._object_order)
        arr = MarkerArray()
        cyl_h = 0.9
        for marker_id, label in enumerate(self._object_order):
            pose_data = self._goals[label]
            x = float(pose_data["x"])
            y = float(pose_data["y"])
            if self._use_fake_discovery:
                show = label in self._discovered_objects
            else:
                show = map_gating.is_goal_known_in_map(self._latest_map, x, y)
            if not show:
                for mid in (marker_id, marker_id + n):
                    m = Marker()
                    m.header.frame_id = self._frame_id
                    m.header.stamp = stamp
                    m.ns = "semantic_objects"
                    m.id = mid
                    m.action = Marker.DELETE
                    arr.markers.append(m)
                continue
            r_vis = float(pose_data["radius"])
            diam = 2.0 * r_vis
            mt = Marker()
            mt.header.frame_id = self._frame_id
            mt.header.stamp = stamp
            mt.ns = "semantic_objects"
            mt.id = marker_id
            mt.type = Marker.TEXT_VIEW_FACING
            mt.action = Marker.ADD
            mt.pose.position.x = x
            mt.pose.position.y = y
            mt.pose.position.z = cyl_h + 0.15
            mt.pose.orientation.w = 1.0
            mt.scale.z = 0.35
            mt.color.r = 0.1
            mt.color.g = 0.85
            mt.color.b = 0.15
            mt.color.a = 1.0
            mt.text = label
            mt.lifetime = BuiltinDuration(sec=0, nanosec=0)
            arr.markers.append(mt)
            ms = Marker()
            ms.header.frame_id = self._frame_id
            ms.header.stamp = stamp
            ms.ns = "semantic_objects"
            ms.id = marker_id + n
            ms.type = Marker.CYLINDER
            ms.action = Marker.ADD
            ms.pose.position.x = x
            ms.pose.position.y = y
            ms.pose.position.z = cyl_h / 2.0
            ms.pose.orientation.w = 1.0
            ms.scale.x = diam
            ms.scale.y = diam
            ms.scale.z = cyl_h
            ms.color.r = 0.2
            ms.color.g = 0.65
            ms.color.b = 0.9
            ms.color.a = 0.6
            ms.lifetime = BuiltinDuration(sec=0, nanosec=0)
            arr.markers.append(ms)
        if arr.markers:
            self._marker_pub.publish(arr)

    def _load_semantic_goals(self) -> tuple[dict, set, str]:
        path_param = self.get_parameter("semantic_goals_file").get_parameter_value().string_value
        path = path_param
        if not path or not os.path.isfile(path):
            try:
                from ament_index_python.packages import get_package_share_directory

                pkg_share = get_package_share_directory("tb3_frontier_exploration")
                path = os.path.join(pkg_share, "config", "semantic_goals.yaml")
            except Exception as e:
                self.get_logger().error("Package share not found: %s" % e)
                return {}, set(), "map"
        if not os.path.isfile(path):
            self.get_logger().error("Semantic goals file not found: %s" % path)
            return {}, set(), "map"
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error("Failed to load YAML %s: %s" % (path, e))
            return {}, set(), "map"
        raw_goals = data.get("semantic_goals") or {}
        goals = {str(k).lower(): raw_goals[k] for k in raw_goals}
        frame_id = data.get("frame_id", "map") or "map"
        return goals, set(goals.keys()), frame_id

    def _coerce_semantic_goal(self, raw: dict) -> dict:
        """Fill defaults from ROS params for missing YAML keys (backward compatible)."""
        r_def = float(self.get_parameter("semantic_object_radius_m").get_parameter_value().double_value)
        c_def = int(self.get_parameter("semantic_arrival_cost_threshold").get_parameter_value().integer_value)
        d_def = float(self.get_parameter("semantic_arrival_xy_tolerance_m").get_parameter_value().double_value)
        return {
            "x": float(raw.get("x", 0.0)),
            "y": float(raw.get("y", 0.0)),
            "yaw": float(raw.get("yaw", 0.0)),
            "radius": float(raw.get("radius", r_def)),
            "arrival_cost_threshold": int(raw.get("arrival_cost_threshold", c_def)),
            "arrival_distance_tolerance": float(raw.get("arrival_distance_tolerance", d_def)),
        }

    def _compute_nav_goal_xy(self, cx: float, cy: float, radius: float) -> tuple[float, float]:
        """
        Nav2 XY target: outside the semantic disk on the near side toward the robot.

        Position = center + (radius + standoff) * u, where u points from center to robot (map frame).
        TF missing or degenerate → +x from center so the goal is never inside the cylinder.
        """
        standoff = max(
            0.0,
            float(self.get_parameter("semantic_nav_goal_standoff_m").get_parameter_value().double_value),
        )
        reach = radius + standoff
        pose = self._get_robot_xy_yaw()
        if pose is None:
            self.get_logger().warning(
                "Nav approach: TF for '%s' in '%s' unavailable after retries; "
                "fallback goal +x from object center, reach=%.3f m."
                % (self._robot_base, self._frame_id, reach)
            )
            return cx + reach, cy
        rx, ry, _ = pose
        dx, dy = rx - cx, ry - cy
        d = math.hypot(dx, dy)
        if d < 1e-6:
            self.get_logger().warning(
                "Nav approach: robot coincident with object center in XY; fallback +x, reach=%.3f m."
                % reach
            )
            return cx + reach, cy
        ux, uy = dx / d, dy / d
        nav_x = cx + reach * ux
        nav_y = cy + reach * uy
        self.get_logger().info(
            "Nav approach: using robot-facing standoff from (%s, %.3f, %.3f) toward '%s' at (%.3f, %.3f)."
            % (self._frame_id, rx, ry, self._robot_base, nav_x, nav_y)
        )
        return nav_x, nav_y

    def _command_callback(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        self.get_logger().info("Command received: '%s'" % raw)

        success, object_label, modifiers, error_message = command_parser.parse_command(
            raw, allowed_objects=self._allowed_objects
        )
        if not success:
            self.get_logger().warn("Command rejected: %s" % (error_message or "unknown"))
            self._publish_result(error_message or "Error")
            return
        if modifiers:
            self.get_logger().info(
                "Object parsed: '%s' (modifiers ignored for nav: %s)" % (object_label, modifiers)
            )
        else:
            self.get_logger().info("Object parsed: '%s'" % object_label)

        if object_label not in self._goals:
            err = "Unknown object '%s'." % object_label
            self.get_logger().warn(err)
            self._publish_result(err)
            return
        g = self._goals[object_label]
        x_c, y_c, yaw = float(g["x"]), float(g["y"]), float(g["yaw"])
        obj_r = float(g["radius"])
        nav_x, nav_y = self._compute_nav_goal_xy(x_c, y_c, obj_r)
        standoff = max(
            0.0,
            float(self.get_parameter("semantic_nav_goal_standoff_m").get_parameter_value().double_value),
        )
        radial = math.hypot(nav_x - x_c, nav_y - y_c)
        want_r = obj_r + standoff
        surf_clear = radial - obj_r
        self.get_logger().info(
            "Semantic region '%s': center=(%.2f, %.2f), r=%.2f, nav_target=(%.2f, %.2f), yaw=%.2f; "
            "nav radial=%.3f (~r+standoff=%.3f), approx clearance beyond cylinder=%.3f m"
            % (object_label, x_c, y_c, obj_r, nav_x, nav_y, yaw, radial, want_r, surf_clear)
        )

        if self._use_map_gating:
            if self._latest_map is None:
                self.get_logger().warn("Map gating: no map received yet.")
                self._publish_result("No map received yet.")
                return
            # Fake discovery proves the semantic disk saw obstacle evidence in /map; the object center
            # cell can still be unknown (-1) or never updated, so a single-point check blocks Nav2.
            if self._use_fake_discovery and object_label in self._discovered_objects:
                self.get_logger().info(
                    "Map gating: '%s' allowed (fake-discovered; skipping center-cell-only check)."
                    % object_label
                )
            elif map_gating.is_goal_known_in_map(self._latest_map, x_c, y_c):
                self.get_logger().info("Map gating: '%s' goal center cell is known." % object_label)
            else:
                self.get_logger().warn(
                    "Map gating: '%s' goal center not known in map (and object not fake-discovered yet)."
                    % object_label
                )
                self._publish_result("Object '%s' not in explored map yet." % object_label)
                return

        # Preempt any running mission
        if self._phase != "idle" or self._nav_goal_handle is not None:
            self.get_logger().warn("Preempting previous semantic mission.")
            self._proximity_cancel = False
            if self._nav_goal_handle is not None:
                self._nav_goal_handle.cancel_goal_async()
            self._publish_twist_zero()
            self._phase = "idle"
            self._mission_pose = None
            self._nav_goal_handle = None

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = self._frame_id
        goal_msg.pose.pose.position.x = nav_x
        goal_msg.pose.pose.position.y = nav_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available.")
            self._publish_result("Nav2 not available.")
            return

        self.get_logger().info("Goal sent: %s -> (%.2f, %.2f)" % (object_label, nav_x, nav_y))
        self._goal_pose_pub.publish(goal_msg.pose)
        self._publish_result("Navigating to %s." % object_label)

        if self._use_proximity:
            self._mission_pose = {
                "cx": x_c,
                "cy": y_c,
                "radius": obj_r,
                "target_yaw": yaw,
                "label": object_label,
                "cost_thresh": int(g["arrival_cost_threshold"]),
                "surface_tol": float(g["arrival_distance_tolerance"]),
            }
            self._phase = "navigate"
            self._proximity_cancel = False

        goal_future = self._action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._goal_accept_callback)

    def _goal_accept_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            if self._use_proximity:
                self._abort_mission_publish("Goal rejected by Nav2.")
            else:
                self._publish_result("Goal rejected by Nav2.")
            return
        self.get_logger().info("Goal accepted by Nav2.")
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_done)

    def _nav2_result_done(self, future) -> None:
        response = future.result()
        status = getattr(response, "status", None)
        if hasattr(status, "status"):
            status = status.status

        if self._proximity_cancel:
            self._proximity_cancel = False
            self.get_logger().info("Nav2 goal ended after proximity preempt; finishing rotate if needed.")
            # rotate phase already running from _mission_tick
            if self._phase == "navigate":
                self._phase = "rotate"
            return

        if not self._use_proximity:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Navigation finished: succeeded.")
                self._publish_result("Navigation finished.")
            else:
                self.get_logger().warn("Navigation finished: failed (status=%s)." % status)
                self._publish_result("Navigation failed (status=%s)." % status)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 reported success; aligning heading.")
            if self._phase == "navigate":
                self._phase = "rotate"
            return

        self.get_logger().warn("Navigation finished: failed (status=%s)." % status)
        self._publish_twist_zero()
        self._phase = "idle"
        self._mission_pose = None
        self._nav_goal_handle = None
        self._publish_result("Navigation failed (status=%s)." % status)

    def _publish_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._result_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSemanticNavigationNode()
    if not node._goals:
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
