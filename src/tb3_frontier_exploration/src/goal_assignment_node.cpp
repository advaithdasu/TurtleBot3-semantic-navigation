#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.hpp>

#include <mutex>
#include <atomic>
#include <cmath>
#include <optional>
#include <vector>

/**
 * @file goal_assignment_node.cpp
 * @brief ROS 2 node: assign Nav2 navigation goals from frontier centroids for autonomous exploration.
 *
 * This node is the bridge between **candidate frontier generation** (frontier_detection_node) and **actual
 * autonomous navigation** (Nav2):
 *
 * - Subscribes to the frontier centroid topic (default `/frontiers`) as `geometry_msgs/PoseArray`, produced by
 *   the frontier detection node.
 * - Obtains the robot pose in the **map** frame using **TF** (`frame_id` → `robot_base_frame`, e.g. map → base_link).
 *   Odometry is subscribed for topic compatibility but pose selection uses TF only in the current implementation.
 * - **Filters** frontier candidates: drops goals too close to the robot and optionally avoids a disk around the
 *   last failed/canceled/aborted goal.
 * - **Selects** the best valid frontier using a **nearest-to-robot** policy among remaining candidates.
 * - Sends **`navigate_to_pose`** goals to Nav2 **asynchronously** via `rclcpp_action`.
 * - Handles **goal timeout** (cancel if navigation stalls), **success** (clears failure memory), **cancellation**,
 *   **rejection**, and **abort** (records position for avoidance on the next selection).
 * - **Records failed goals** so `selectBestFrontier` can skip centroids near recent failures.
 *
 * Together with frontier_detection_node, this closes the exploration loop: map → frontiers → Nav2 → motion → updated map.
 */

namespace tb3_frontier_exploration
{

enum class State { IDLE, NAVIGATING };

class GoalAssignmentNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Construct the goal assignment node: parameters, TF, subscriptions, Nav2 action client, and control timer.
   *
   * @param none No constructor arguments; all behavior is configured via ROS parameters.
   * @return N/A (constructs the node in place).
   *
   * Pipeline role:
   * - Connects the exploration executive to `/frontiers` and Nav2’s `navigate_to_pose` action; without this setup,
   *   frontier centroids would never become motion commands.
   *
   * Implementation summary:
   * 1. Declares parameters for frontier and odom topic names, Nav2 action name, map/base frames, timeouts,
   *    selection distances, timer rate, and exploration-complete log throttling.
   * 2. Creates `tf2_ros::Buffer` and `TransformListener` for map-frame robot position.
   * 3. Subscribes to `PoseArray` frontiers and `Odometry` (odom callback is currently a no-op).
   * 4. Creates the `NavigateToPose` action client and a wall timer at `rate` Hz to run `timerCallback`.
   * 5. Logs startup configuration (topics, distances, timeout).
   *
   * Notes:
   * - Initial FSM state is `IDLE`; the timer drives both idle goal dispatch and navigating timeout checks.
   * - Action server must be available (Nav2 bt_navigator) before goals are sent.
   */
  GoalAssignmentNode()
  : Node("goal_assignment_node"),
    state_(State::IDLE)
  {
    declare_parameter<std::string>("frontiers_topic", "/frontiers");
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("navigate_to_pose_action", "navigate_to_pose");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("robot_base_frame", "base_link");
    declare_parameter<double>("goal_timeout", 60.0);
    declare_parameter<double>("rate", 1.0);
    declare_parameter<double>("min_frontier_distance", 0.5);
    declare_parameter<double>("failed_goal_avoidance_radius", 1.0);
    declare_parameter<double>("exploration_complete_log_interval", 5.0);
    declare_parameter<bool>("require_startup_warmup", false);
    declare_parameter<std::string>("startup_warmup_complete_topic", "exploration_warmup_complete");
    declare_parameter<double>("startup_warmup_timeout_sec", 90.0);
    declare_parameter<bool>("enable_repeated_centroid_filter", true);
    declare_parameter<double>("repeated_centroid_reject_radius", 0.45);
    declare_parameter<int>("repeated_centroid_max_visits", 2);

    std::string frontiers_topic = get_parameter("frontiers_topic").as_string();
    std::string odom_topic = get_parameter("odom_topic").as_string();
    std::string action_name = get_parameter("navigate_to_pose_action").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    frontiers_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      frontiers_topic, 10, std::bind(&GoalAssignmentNode::frontiersCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&GoalAssignmentNode::odomCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name);

    require_startup_warmup_ = get_parameter("require_startup_warmup").as_bool();
    startup_warmup_done_ = !require_startup_warmup_;
    warmup_watch_start_ = now();
    std::string warmup_topic = get_parameter("startup_warmup_complete_topic").as_string();
    warmup_sub_ = create_subscription<std_msgs::msg::Bool>(
      warmup_topic, rclcpp::QoS(10),
      std::bind(&GoalAssignmentNode::warmupCompleteCallback, this, std::placeholders::_1));

    exploration_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      "exploration_enabled", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        exploration_enabled_ = msg->data;
        RCLCPP_INFO(get_logger(), "[exploration] %s via /exploration_enabled",
          exploration_enabled_ ? "ENABLED" : "DISABLED");
      });

    double rate_hz = get_parameter("rate").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&GoalAssignmentNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "[goal_assignment] Started: frontiers=%s, action=%s, min_frontier_dist=%.2f, failed_avoid_radius=%.2f, goal_timeout=%.1fs",
      frontiers_topic.c_str(), action_name.c_str(),
      get_parameter("min_frontier_distance").as_double(),
      get_parameter("failed_goal_avoidance_radius").as_double(),
      get_parameter("goal_timeout").as_double());
    if (require_startup_warmup_) {
      RCLCPP_INFO(get_logger(),
        "[goal_assignment] Startup warmup required; waiting for Bool true on '%s' (or timeout %.1fs)",
        warmup_topic.c_str(), get_parameter("startup_warmup_timeout_sec").as_double());
    }
  }

private:
  void warmupCompleteCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      startup_warmup_done_ = true;
      RCLCPP_INFO(get_logger(), "[warmup] Received complete signal — frontier exploration enabled.");
    }
  }

  /**
   * @brief Store the latest frontier centroid message from frontier_detection_node under a mutex.
   *
   * @param msg Shared pointer to `geometry_msgs/PoseArray` (each pose’s position is a frontier centroid in map frame).
   * @return void; updates member `latest_frontiers_`.
   *
   * Pipeline role:
   * - Provides the **input queue** of exploration candidates for the timer-driven executive; without fresh messages,
   *   the node has nothing to navigate toward.
   *
   * Implementation summary:
   * 1. Lock `frontiers_mutex_`.
   * 2. Assign `latest_frontiers_ = msg` (shared pointer copy, replaces previous snapshot).
   * 3. Optionally log centroid count at DEBUG level.
   *
   * Notes:
   * - Each callback **replaces** the entire cached array when frontier_detection republishes on map update.
   * - Thread-safe with `timerCallback`, which reads the same pointer under the same mutex.
   */
  void frontiersCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(frontiers_mutex_);
    latest_frontiers_ = msg;
    RCLCPP_DEBUG(get_logger(), "[frontiers] Received %zu centroids", msg->poses.size());
  }

  /**
   * @brief Odometry subscription callback (currently unused for pose or control).
   *
   * @param msg Latest `nav_msgs/Odometry` on `odom_topic` (ignored in the body).
   * @return void.
   *
   * Pipeline role:
   * - Reserved hook for future use (e.g. velocity checks or fallback pose); **does not** participate in frontier
   *   selection today—robot position comes from TF in `getRobotPoseInMap`.
   *
   * Implementation summary:
   * 1. Cast `msg` to void to silence unused-parameter warnings.
   *
   * Notes:
   * - Keeping the subscription allows the node to match launch files that expect an odom topic without extra nodes.
   * - Do not assume pose is updated here when reading or extending the code.
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    (void)msg;
  }

  /**
   * @brief Look up the robot base frame position projected into the map plane using TF.
   *
   * @param rx Output: robot x in `frame_id` (map) frame, meters.
   * @param ry Output: robot y in `frame_id` (map) frame, meters.
   * @return `true` if `lookupTransform(frame_id, robot_base_frame, ...)` succeeds; `false` on TF failure (outputs undefined).
   *
   * Pipeline role:
   * - Enables **distance-based** frontier filtering and **nearest-neighbor** selection; without a valid transform,
   *   the timer callback aborts before sending a goal.
   *
   * Implementation summary:
   * 1. Read `frame_id` and `robot_base_frame` from parameters.
   * 2. Call `tf_buffer_->lookupTransform` with latest time (`TimePointZero`) and 0.5 s timeout.
   * 3. On success, set `rx`, `ry` from `transform.translation` (x, y); on exception, log at DEBUG and return false.
   *
   * Notes:
   * - Only **translation** is used; yaw is not needed for Euclidean distance in the plane.
   * - Requires TF tree from `robot_base_frame` to `frame_id` to be published (e.g. map → odom → base_link).
   */
  bool getRobotPoseInMap(double & rx, double & ry)
  {
    std::string frame_id = get_parameter("frame_id").as_string();
    std::string robot_frame = get_parameter("robot_base_frame").as_string();
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
        frame_id, robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
      rx = t.transform.translation.x;
      ry = t.transform.translation.y;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(get_logger(), "[TF] Lookup failed: %s", e.what());
      return false;
    }
  }

  /**
   * @brief Choose the nearest valid frontier index for the robot, applying distance and failure-avoidance filters.
   *
   * @param frontiers `PoseArray` of candidate centroids (positions in map frame; header may supply `frame_id`).
   * @param rx Robot x in map frame (meters), from TF.
   * @param ry Robot y in map frame (meters), from TF.
   * @param frame_id In/out: goal frame string; if empty on input, set from `frontiers.header.frame_id` when a candidate exists.
   * @return Index of the selected pose in `frontiers.poses`, or `std::nullopt` if every candidate is filtered out.
   *
   * Pipeline role:
   * - Implements the exploration **policy** (greedy nearest valid frontier); bridges perception output to a single
   *   actionable goal per timer cycle.
   *
   * Implementation summary:
   * 1. Load `min_frontier_distance` and `failed_goal_avoidance_radius` from parameters (squared for comparisons).
   * 2. For each frontier pose, compute squared distance to `(rx, ry)`; skip if closer than `min_frontier_distance`.
   * 3. If `last_failed_goal_` is set, skip centroids within `failed_goal_avoidance_radius` of that point.
   * 4. Among remaining, track the index with minimum squared distance.
   * 5. If none remain, return nullopt; else if `frame_id` was empty, copy `frontiers.header.frame_id`, then return index.
   *
   * Notes:
   * - **Nearest** is Euclidean in the map **xy** plane; z is ignored.
   * - Ties are broken by **first** minimum encountered (order is array order).
   */
  std::optional<size_t> selectBestFrontier(
    const geometry_msgs::msg::PoseArray & frontiers,
    double rx, double ry,
    std::string & frame_id)
  {
    const double min_dist = get_parameter("min_frontier_distance").as_double();
    const double min_dist_sq = min_dist * min_dist;
    const double avoid_radius = get_parameter("failed_goal_avoidance_radius").as_double();
    const double avoid_radius_sq = avoid_radius * avoid_radius;

    size_t best = 0;
    double best_dist_sq = 1e30;
    bool found = false;

    for (size_t i = 0; i < frontiers.poses.size(); i++) {
      double gx = frontiers.poses[i].position.x;
      double gy = frontiers.poses[i].position.y;
      double dx = gx - rx;
      double dy = gy - ry;
      double d2 = dx * dx + dy * dy;

      if (d2 < min_dist_sq) {
        RCLCPP_DEBUG(get_logger(), "[select] Frontier [%zu] (%.2f, %.2f) skipped: too close (%.2f m)",
          i, gx, gy, std::sqrt(d2));
        continue;
      }
      if (last_failed_goal_) {
        double fdx = gx - last_failed_goal_->x;
        double fdy = gy - last_failed_goal_->y;
        if (fdx * fdx + fdy * fdy < avoid_radius_sq) {
          RCLCPP_DEBUG(get_logger(), "[select] Frontier [%zu] (%.2f, %.2f) skipped: near last failed goal",
            i, gx, gy);
          continue;
        }
      }

      if (get_parameter("enable_repeated_centroid_filter").as_bool()) {
        const double loop_r = get_parameter("repeated_centroid_reject_radius").as_double();
        const int max_vis = get_parameter("repeated_centroid_max_visits").as_int();
        if (loop_r > 0.0 && max_vis > 0) {
          const double loop_r_sq = loop_r * loop_r;
          const size_t n_vis = countSuccessfulVisitsNear(gx, gy, loop_r_sq);
          if (static_cast<int>(n_vis) >= max_vis) {
            RCLCPP_DEBUG(get_logger(),
              "[select] Frontier [%zu] (%.2f, %.2f) skipped: anti-loop (>= %d visits within %.2f m)",
              i, gx, gy, max_vis, loop_r);
            continue;
          }
        }
      }

      if (d2 < best_dist_sq) {
        best_dist_sq = d2;
        best = i;
        found = true;
      }
    }

    if (!found) return std::nullopt;
    if (frame_id.empty()) frame_id = frontiers.header.frame_id;
    return best;
  }

  /**
   * @brief Periodic executive: enforce Nav2 goal timeout when navigating; when idle, select and send the next frontier goal.
   *
   * @param none.
   * @return void; may call `async_cancel_goal` or `async_send_goal` as side effects.
   *
   * Pipeline role:
   * - **Central loop** for closed-loop exploration: supervises the current Nav2 goal and dispatches the next one
   *   when safe; without this timer, no goals would be issued after startup.
   *
   * Implementation summary:
   * 1. If state is `NAVIGATING` and a goal handle and send time exist: if elapsed time ≥ `goal_timeout`, record
   *    failure at `current_goal_x_/y_`, clear handle/time, set `IDLE`, call `async_cancel_goal`, return.
   * 2. If still `NAVIGATING` (no timeout), return (do not send another goal).
   * 3. If Nav2 action server not ready, return.
   * 4. Copy `latest_frontiers_` under mutex; if null or empty, call `logExplorationCompleteIfDue` and return.
   * 5. `getRobotPoseInMap`; on failure return.
   * 6. `selectBestFrontier`; if none, log and `logExplorationCompleteIfDue`, return.
   * 7. Log selection; erase chosen pose from `latest_frontiers_->poses` under mutex (avoid immediate duplicate from cache).
   * 8. Build `NavigateToPose::Goal` with identity quaternion; set `current_goal_x_/y_`; set state `NAVIGATING`;
   *    register `goal_response_callback` and `result_callback`; call `async_send_goal`.
   *
   * Notes:
   * - State is set to `NAVIGATING` **before** acceptance; if the server **rejects**, the response callback resets to `IDLE`.
   * - Timeout uses time **after** acceptance (`goal_sent_time_` set in response callback), not wall time from send call.
   * - Cancel path copies handle before reset so `async_cancel_goal` still receives a valid handle.
   */
  void timerCallback()
  {
    const double goal_timeout = get_parameter("goal_timeout").as_double();

    if (state_ == State::NAVIGATING) {
      if (current_goal_handle_ && goal_sent_time_) {
        double elapsed = (now() - *goal_sent_time_).seconds();
        if (elapsed >= goal_timeout) {
          RCLCPP_WARN(get_logger(), "[timeout] Goal stalled for %.1fs (limit %.1fs), canceling",
            elapsed, goal_timeout);
          auto handle = current_goal_handle_;
          current_goal_handle_.reset();
          goal_sent_time_.reset();
          recordFailedGoal(current_goal_x_, current_goal_y_);
          state_ = State::IDLE;
          if (handle) {
            nav_client_->async_cancel_goal(handle);
          }
        }
      }
      return;
    }

    if (!exploration_enabled_) {
      return;
    }

    if (require_startup_warmup_ && !startup_warmup_done_) {
      const double tout = get_parameter("startup_warmup_timeout_sec").as_double();
      if ((now() - warmup_watch_start_).seconds() >= tout) {
        if (!warmup_timeout_announced_) {
          RCLCPP_WARN(get_logger(),
            "[warmup] No complete signal after %.1fs — enabling exploration anyway.", tout);
          warmup_timeout_announced_ = true;
        }
        startup_warmup_done_ = true;
      } else {
        if (!last_warmup_waiting_log_ ||
          (now() - *last_warmup_waiting_log_).seconds() >= 5.0)
        {
          RCLCPP_INFO(get_logger(), "[warmup] Waiting for startup map warmup…");
          last_warmup_waiting_log_ = now();
        }
        return;
      }
    }

    if (!nav_client_->action_server_is_ready()) {
      RCLCPP_DEBUG(get_logger(), "[idle] Action server not ready");
      return;
    }

    geometry_msgs::msg::PoseArray::SharedPtr frontiers;
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      frontiers = latest_frontiers_;
    }

    if (!frontiers || frontiers->poses.empty()) {
      logExplorationCompleteIfDue();
      return;
    }

    double rx, ry;
    if (!getRobotPoseInMap(rx, ry)) return;

    std::string frame_id = get_parameter("frame_id").as_string();
    std::optional<size_t> best_opt = selectBestFrontier(*frontiers, rx, ry, frame_id);

    if (!best_opt) {
      RCLCPP_DEBUG(get_logger(), "[select] No valid frontier after filters (min_dist=%.2f, avoid_radius=%.2f)",
        get_parameter("min_frontier_distance").as_double(),
        get_parameter("failed_goal_avoidance_radius").as_double());
      logExplorationCompleteIfDue();
      return;
    }

    size_t best = *best_opt;
    double gx = frontiers->poses[best].position.x;
    double gy = frontiers->poses[best].position.y;
    double dist = std::sqrt(
      (gx - rx) * (gx - rx) + (gy - ry) * (gy - ry));

    RCLCPP_INFO(get_logger(), "[goal] Selected [%zu] (%.2f, %.2f) dist=%.2f m",
      best, gx, gy, dist);

    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      if (latest_frontiers_ && best < latest_frontiers_->poses.size()) {
        latest_frontiers_->poses.erase(latest_frontiers_->poses.begin() + best);
      }
    }

    NavigateToPose::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = frame_id;
    goal.pose.pose.position.x = gx;
    goal.pose.pose.position.y = gy;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;

    current_goal_x_ = gx;
    current_goal_y_ = gy;

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_opts.goal_response_callback = [this](typename GoalHandle::SharedPtr gh) {
      if (gh) {
        RCLCPP_INFO(get_logger(), "[action] Goal accepted");
        current_goal_handle_ = gh;
        goal_sent_time_ = now();
        state_ = State::NAVIGATING;
      } else {
        RCLCPP_WARN(get_logger(), "[action] Goal rejected");
        recordFailedGoal(current_goal_x_, current_goal_y_);
        state_ = State::IDLE;
      }
    };
    send_opts.result_callback = [this](const GoalHandle::WrappedResult & result) {
      current_goal_handle_.reset();
      goal_sent_time_.reset();
      state_ = State::IDLE;

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "[result] SUCCEEDED");
          last_failed_goal_.reset();
          recordSuccessfulVisit(current_goal_x_, current_goal_y_);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(get_logger(), "[result] ABORTED");
          recordFailedGoal(current_goal_x_, current_goal_y_);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(get_logger(), "[result] CANCELED");
          recordFailedGoal(current_goal_x_, current_goal_y_);
          break;
        default:
          RCLCPP_WARN(get_logger(), "[result] Unknown code");
          recordFailedGoal(current_goal_x_, current_goal_y_);
      }
    };

    state_ = State::NAVIGATING;
    nav_client_->async_send_goal(goal, send_opts);
  }

  /**
   * @brief Remember a world-frame goal position that failed, timed out, or was rejected so selection can avoid it.
   *
   * @param gx Goal x in map frame (meters), same as sent to Nav2.
   * @param gy Goal y in map frame (meters).
   * @return void; writes `last_failed_goal_`.
   *
   * Pipeline role:
   * - Provides **simple recovery**: reduces repeated attempts at the same bad centroid (obstacle, planner failure, etc.).
   *
   * Implementation summary:
   * 1. Assign `last_failed_goal_` to `{gx, gy}`.
   * 2. Log at DEBUG.
   *
   * Notes:
   * - Cleared on **SUCCEEDED** in `result_callback`; not cleared on mere timeout cancel until the next outcome handling.
   * - Only **one** failure point is stored; a new failure overwrites the previous avoidance center.
   */
  void recordFailedGoal(double gx, double gy)
  {
    last_failed_goal_ = {gx, gy};
    RCLCPP_DEBUG(get_logger(), "[failed] Recorded (%.2f, %.2f) for avoidance", gx, gy);
  }

  /** Count how many successful frontier goals landed within radius (map plane) of (mx, my). */
  size_t countSuccessfulVisitsNear(double mx, double my, double radius_sq) const
  {
    size_t n = 0;
    for (const auto & p : visit_history_) {
      const double dx = mx - p.x;
      const double dy = my - p.y;
      if (dx * dx + dy * dy <= radius_sq) {
        ++n;
      }
    }
    return n;
  }

  void recordSuccessfulVisit(double gx, double gy)
  {
    const size_t k_max_history = 256;
    visit_history_.push_back({gx, gy});
    while (visit_history_.size() > k_max_history) {
      visit_history_.erase(visit_history_.begin());
    }
    RCLCPP_DEBUG(get_logger(), "[antiloop] Recorded reached centroid (%.2f, %.2f), history=%zu",
      gx, gy, visit_history_.size());
  }

  /**
   * @brief Log a throttled INFO message when no valid frontiers are available (exploration appears complete).
   *
   * @param none Uses `exploration_complete_log_interval` parameter and `last_exploration_complete_log_time_`.
   * @return void.
   *
   * Pipeline role:
   * - Gives operators a clear **status** when the robot is idle due to empty or filtered frontier lists, without
   *   flooding logs on every timer tick.
   *
   * Implementation summary:
   * 1. Read `exploration_complete_log_interval` (seconds).
   * 2. If no prior log time, or `now` minus last log ≥ interval, emit INFO and store `now` as last log time.
   *
   * Notes:
   * - Exploration may **resume** automatically when frontier_detection publishes new centroids; the message states this.
   */
  void logExplorationCompleteIfDue()
  {
    double interval = get_parameter("exploration_complete_log_interval").as_double();
    auto now_t = now();
    if (!last_exploration_complete_log_time_ ||
        (now_t - *last_exploration_complete_log_time_).seconds() >= interval) {
      RCLCPP_INFO(get_logger(), "[exploration] No valid frontiers — exploration complete (will retry when new frontiers appear)");
      last_exploration_complete_log_time_ = now_t;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr frontiers_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr warmup_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_enabled_sub_;
  bool exploration_enabled_{true};
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseArray::SharedPtr latest_frontiers_;
  std::mutex frontiers_mutex_;

  State state_;
  typename GoalHandle::SharedPtr current_goal_handle_;
  std::optional<rclcpp::Time> goal_sent_time_;
  double current_goal_x_{0.0};
  double current_goal_y_{0.0};

  struct Point { double x, y; };
  std::optional<Point> last_failed_goal_;
  std::optional<rclcpp::Time> last_exploration_complete_log_time_;

  std::vector<Point> visit_history_;

  bool require_startup_warmup_{true};
  bool startup_warmup_done_{false};
  rclcpp::Time warmup_watch_start_;
  bool warmup_timeout_announced_{false};
  std::optional<rclcpp::Time> last_warmup_waiting_log_;
};

}  // namespace tb3_frontier_exploration

/**
 * @brief Program entry: initialize ROS 2 and spin one `GoalAssignmentNode` until shutdown.
 *
 * @param argc Standard C argument count.
 * @param argv Standard C argument vector (ROS 2 remapping may apply).
 * @return 0 after `rclcpp::shutdown`.
 *
 * Pipeline role:
 * - Starts the goal-assignment executable so the exploration stack can receive Nav2 action results and frontier updates.
 *
 * Implementation summary:
 * 1. `rclcpp::init(argc, argv)`.
 * 2. `rclcpp::spin` on a shared `GoalAssignmentNode`.
 * 3. `rclcpp::shutdown()`; return 0.
 *
 * Notes:
 * - Single-threaded executor: timer, subscriptions, and action callbacks are serialized unless a multi-threaded spinner is used elsewhere.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tb3_frontier_exploration::GoalAssignmentNode>());
  rclcpp::shutdown();
  return 0;
}
