/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  // Input/output topic names (config-driven). ROS remapping still applies on top of these.
  cloud_topic_ = this->declare_parameter("cloud_topic", "cloud_in");
  scan_topic_ = this->declare_parameter("scan_topic", "scan");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);

  // Named height profiles (see header). Each profile's slab defaults to the base slab.
  base_slab_ = {min_height_, max_height_};
  for (const auto & name : this->declare_parameter(
      "height_profiles", std::vector<std::string>{}))
  {
    if (name.empty() || name == kDefaultProfile || name.find('.') != std::string::npos) {
      RCLCPP_WARN(this->get_logger(), "height_profiles: '%s' ignored (reserved/invalid)", name.c_str());
      continue;
    }
    const double lo = this->declare_parameter(name + ".min_height", base_slab_.first);
    const double hi = this->declare_parameter(name + ".max_height", base_slab_.second);
    if (lo >= hi) {
      throw std::invalid_argument(
              "height profile '" + name + "': min_height must be < max_height");
    }
    profiles_[name] = {lo, hi};
    RCLCPP_INFO(this->get_logger(), "Height profile '%s': [%.3f, %.3f] m", name.c_str(), lo, hi);
  }
  active_profile_ = this->declare_parameter("height_profile", std::string(kDefaultProfile));
  if (active_profile_ != kDefaultProfile) {
    auto it = profiles_.find(active_profile_);
    if (it == profiles_.end()) {
      throw std::invalid_argument(
              "height_profile '" + active_profile_ + "' is not in height_profiles");
    }
    std::tie(min_height_, max_height_) = it->second;
  }
  RCLCPP_INFO(
    this->get_logger(), "Height slab [%s]: [%.3f, %.3f] m", active_profile_.c_str(),
    min_height_, max_height_);

  // Axis-aligned self-filter on x/y: drop points INSIDE the box (robot body).
  // Defaults to a zero-size box so an accidental enable removes nothing.
  use_self_filter_ = this->declare_parameter("use_self_filter", false);
  self_x_min_ = this->declare_parameter("self_x_min", 0.0);
  self_x_max_ = this->declare_parameter("self_x_max", 0.0);
  self_y_min_ = this->declare_parameter("self_y_min", 0.0);
  self_y_max_ = this->declare_parameter("self_y_max", 0.0);

  // Footprint-topic self-filter (geometry_msgs/PolygonStamped), unioned with the box above.
  // Empty topic = box only. Stale (> timeout) or untransformable polygon = box only.
  self_footprint_topic_ = this->declare_parameter("self_footprint_topic", "");
  self_footprint_timeout_ = this->declare_parameter("self_footprint_timeout", 0.5);
  self_footprint_padding_ = this->declare_parameter("self_footprint_padding", 0.04);

  // Livox tag noise filter: drop a point when (tag & tag_filter_mask_) != 0.
  tag_filter_enable_ = this->declare_parameter("tag_filter_enable", false);
  tag_filter_mask_ = this->declare_parameter("tag_filter_mask", 0x0F);

  // Sparse near-field ghost filter. Center = sensor origin in target_frame.
  ghost_filter_enable_ = this->declare_parameter("ghost_filter_enable", false);
  ghost_params_.center_x = this->declare_parameter("ghost_filter_center_x", 0.0);
  ghost_params_.center_y = this->declare_parameter("ghost_filter_center_y", 0.0);
  ghost_params_.center_z = this->declare_parameter("ghost_filter_center_z", 0.0);
  ghost_params_.radius_ratio = this->declare_parameter("ghost_filter_radius_ratio", 0.10);
  ghost_params_.min_radius = this->declare_parameter("ghost_filter_min_radius", 0.02);
  ghost_params_.min_neighbors = static_cast<int>(
    this->declare_parameter("ghost_filter_min_neighbors", 8));
  ghost_params_.max_check_range = this->declare_parameter("ghost_filter_max_check_range", 1.5);
  ghost_params_.behind_distance = this->declare_parameter("ghost_filter_behind_distance", 0.15);
  if (ghost_filter_enable_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Ghost filter: points within %.2f m of (%.3f, %.3f, %.3f) need >= %d neighbors within "
      "max(%.3f, %.3f * r); a sparse nearest point yields to a supported one <= %.2f m behind.",
      ghost_params_.max_check_range, ghost_params_.center_x, ghost_params_.center_y,
      ghost_params_.center_z, ghost_params_.min_neighbors, ghost_params_.min_radius,
      ghost_params_.radius_ratio, ghost_params_.behind_distance);
  }

  declareHeightZones();

  // Registered after all declarations so it only sees runtime changes.
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PointCloudToLaserScanNode::onParametersSet, this, std::placeholders::_1));

  pub_ =this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, rclcpp::SensorDataQoS());

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  if (use_self_filter_ && !self_footprint_topic_.empty()) {
    if (target_frame_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "self_footprint_topic '%s' needs target_frame for TF; using fixed self box only.",
        self_footprint_topic_.c_str());
    } else {
      footprint_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        self_footprint_topic_, rclcpp::QoS(1),
        std::bind(&PointCloudToLaserScanNode::footprintCallback, this, _1));
      RCLCPP_INFO(
        this->get_logger(),
        "Self filter: fixed box UNION footprint '%s' (padding %.3f m, timeout %.2f s). "
        "Fixed box only until the footprint arrives.",
        self_footprint_topic_.c_str(), self_footprint_padding_, self_footprint_timeout_);
    }
  }

  if (!zones_.empty()) {
    startHeightZones();
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

void PointCloudToLaserScanNode::declareHeightZones()
{
  zone_robot_frame_ = this->declare_parameter("height_zone_robot_frame", "base_link");
  zone_floor_timeout_ = this->declare_parameter("height_zone_floor_timeout", 3.0);
  zone_lookup_period_ = this->declare_parameter("height_zone_lookup_period", 30.0);
  zone_lookup_service_ = this->declare_parameter(
    "height_zone_lookup_service", "/db_manager/find_destination");
  for (const auto & name : this->declare_parameter(
      "height_zones", std::vector<std::string>{}))
  {
    height_zone::Zone z;
    z.name = name;
    if (name.empty() || name.find('.') != std::string::npos) {
      RCLCPP_WARN(this->get_logger(), "height_zones: '%s' ignored (invalid name)", name.c_str());
      continue;
    }
    z.point_code = this->declare_parameter(name + ".point_code", "");
    z.profile = this->declare_parameter(name + ".profile", "");
    z.radius = this->declare_parameter(name + ".radius", 3.0);
    z.exit_margin = this->declare_parameter(name + ".exit_margin", 0.3);
    z.frame = this->declare_parameter(name + ".frame", "map");
    std::string bad;
    if (z.point_code.empty()) {
      bad = "point_code is empty";
    } else if (profiles_.find(z.profile) == profiles_.end()) {
      bad = "profile '" + z.profile + "' is not in height_profiles";
    } else if (!(z.radius > 0.0) || !(z.exit_margin >= 0.0)) {
      bad = "radius must be > 0 and exit_margin >= 0";
    }
    if (!bad.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Height zone '%s' skipped: %s", name.c_str(), bad.c_str());
      continue;
    }
    zones_.push_back(z);
  }
  zone_pending_.assign(zones_.size(), false);
  zone_warned_.assign(zones_.size(), false);
}

void PointCloudToLaserScanNode::startHeightZones()
{
  if (!tf2_) {  // target_frame empty -> no TF yet
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
  }
  // One mutually exclusive group: tick, floor and lookup responses never run concurrently.
  zone_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = zone_cb_group_;
  const auto floor_topic = this->declare_parameter("height_zone_floor_topic", "/current_floor");
  floor_sub_ = this->create_subscription<w_floor_perception_msgs::msg::FloorInfo>(
    floor_topic, rclcpp::QoS(10),
    [this](w_floor_perception_msgs::msg::FloorInfo::ConstSharedPtr msg) {
      floor_label_ = height_zone::normalizeFloor(msg->floor_label);
      floor_stamp_ = std::chrono::steady_clock::now();
    }, sub_opts);
  zone_client_ = this->create_client<FindDestination>(
    zone_lookup_service_, rclcpp::ServicesQoS(), zone_cb_group_);
  next_lookup_ = std::chrono::steady_clock::now();
  const double period = this->declare_parameter("height_zone_period", 0.5);
  zone_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period), std::bind(&PointCloudToLaserScanNode::heightZoneTick, this),
    zone_cb_group_);
  for (const auto & z : zones_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Height zone '%s': point '%s' (DB) radius %.2f m, exit +%.2f m, frame %s -> profile '%s'. "
      "Zones own height_profile; manual sets are rejected.",
      z.name.c_str(), z.point_code.c_str(), z.radius, z.exit_margin, z.frame.c_str(),
      z.profile.c_str());
  }
}

void PointCloudToLaserScanNode::requestZoneLookups()
{
  zone_client_->prune_pending_requests();  // drop answers that never came
  if (!zone_client_->service_is_ready()) {
    if (!zone_service_warned_) {
      RCLCPP_WARN(
        this->get_logger(), "Height zones: service %s unavailable; unresolved zones stay off "
        "(retry every %.0f s).", zone_lookup_service_.c_str(), zone_lookup_period_);
      zone_service_warned_ = true;
    }
    return;
  }
  zone_service_warned_ = false;
  for (size_t i = 0; i < zones_.size(); ++i) {
    if (zones_[i].resolved) {
      continue;
    }
    auto req = std::make_shared<FindDestination::Request>();
    req->destination = zones_[i].point_code;
    req->use_floor = false;
    zone_client_->async_send_request(
      req, [this, i](rclcpp::Client<FindDestination>::SharedFuture f) {
        const auto r = f.get();
        auto & z = zones_[i];
        if (!r->found) {
          if (!zone_warned_[i]) {
            RCLCPP_WARN(
              this->get_logger(), "Height zone '%s': point '%s' not found in robot DB "
              "(%zu candidates); zone skipped, retry every %.0f s.", z.name.c_str(),
              z.point_code.c_str(), r->candidates.size(), zone_lookup_period_);
            zone_warned_[i] = true;
          }
          return;
        }
        z.x = r->x;
        z.y = r->y;
        z.floor = height_zone::normalizeFloor(r->floor);
        z.resolved = true;
        RCLCPP_INFO(
          this->get_logger(), "Height zone '%s': point '%s' = (%.3f, %.3f) floor '%s' [%s]",
          z.name.c_str(), z.point_code.c_str(), z.x, z.y, z.floor.c_str(), z.frame.c_str());
        if (z.floor.empty()) {
          RCLCPP_WARN(
            this->get_logger(), "Height zone '%s': DB point has no floor; zone never matches.",
            z.name.c_str());
        }
      });
  }
}

void PointCloudToLaserScanNode::heightZoneTick()
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= next_lookup_) {
    next_lookup_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(zone_lookup_period_));
    requestZoneLookups();
  }

  height_zone::Observation obs;
  obs.floor_ok = !floor_label_.empty() &&
    std::chrono::duration<double>(now - floor_stamp_).count() < zone_floor_timeout_;
  obs.floor = floor_label_;
  obs.poses.resize(zones_.size());
  for (size_t i = 0; i < zones_.size(); ++i) {
    if (!zones_[i].resolved) {
      continue;
    }
    try {
      const auto t = tf2_->lookupTransform(
        zones_[i].frame, zone_robot_frame_, tf2::TimePointZero).transform.translation;
      obs.poses[i] = {true, t.x, t.y};
    } catch (const tf2::TransformException & e) {
      if (!zone_tf_warned_) {
        RCLCPP_WARN(
          this->get_logger(), "Height zones: TF %s->%s unavailable (%s); keeping profile.",
          zones_[i].frame.c_str(), zone_robot_frame_.c_str(), e.what());
        zone_tf_warned_ = true;
      }
    }
  }

  const auto d = height_zone::decide(zones_, active_zone_, obs);
  if (d.zone != active_zone_) {
    const std::string floor = obs.floor_ok ? obs.floor : "?";
    if (active_zone_ >= 0) {
      const auto & z = zones_[active_zone_];
      const auto & p = obs.poses[active_zone_];
      RCLCPP_INFO(
        this->get_logger(), "Height zone '%s' left (%.2f m > %.2f m or other floor, floor %s)",
        z.name.c_str(), std::hypot(p.x - z.x, p.y - z.y), z.radius + z.exit_margin,
        floor.c_str());
    }
    if (d.zone >= 0) {
      RCLCPP_INFO(
        this->get_logger(), "Height zone '%s' entered (%.2f m <= %.2f m, floor %s)",
        zones_[d.zone].name.c_str(), d.distance, zones_[d.zone].radius, floor.c_str());
    }
    active_zone_ = d.zone;
  }

  const std::string want = active_zone_ >= 0 ? zones_[active_zone_].profile : kDefaultProfile;
  {
    std::lock_guard<std::mutex> lock(height_mutex_);
    if (want == active_profile_) {
      return;
    }
  }
  zone_set_thread_.store(std::this_thread::get_id());
  const auto r = this->set_parameter(rclcpp::Parameter("height_profile", want));
  zone_set_thread_.store(std::thread::id{});
  if (!r.successful) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "Height zones: set height_profile '%s' "
      "failed: %s", want.c_str(), r.reason.c_str());
  }
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, cloud_topic_, qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

rcl_interfaces::msg::SetParametersResult PointCloudToLaserScanNode::onParametersSet(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(height_mutex_);
  auto base = base_slab_;
  auto profiles = profiles_;
  auto active = active_profile_;
  bool touched = false;
  for (const auto & p : parameters) {
    const std::string & n = p.get_name();
    if (n == "min_height") {
      base.first = p.as_double();
    } else if (n == "max_height") {
      base.second = p.as_double();
    } else if (n == "height_profile") {
      if (!zones_.empty() && zone_set_thread_.load() != std::this_thread::get_id()) {
        result.successful = false;
        result.reason = "height_zones is on: the zone decision owns height_profile (now '" +
          active_profile_ + "')";
        RCLCPP_WARN(
          this->get_logger(), "Manual height_profile '%s' overridden (rejected): %s",
          p.as_string().c_str(), result.reason.c_str());
        return result;
      }
      active = p.as_string();
    } else {
      // <profile>.min_height / <profile>.max_height of a declared profile
      const auto dot = n.rfind('.');
      if (dot == std::string::npos) {
        continue;
      }
      auto it = profiles.find(n.substr(0, dot));
      const std::string key = n.substr(dot + 1);
      if (it == profiles.end() || (key != "min_height" && key != "max_height")) {
        continue;
      }
      (key == "min_height" ? it->second.first : it->second.second) = p.as_double();
    }
    touched = true;
  }
  if (!touched) {
    return result;
  }
  // An inverted slab would publish an all-empty scan, which clears obstacles downstream.
  auto inverted = [&result](const std::string & what, const std::pair<double, double> & slab) {
      if (slab.first < slab.second) {
        return false;
      }
      result.successful = false;
      result.reason = what + ": min_height (" + std::to_string(slab.first) +
        ") must be < max_height (" + std::to_string(slab.second) + ")";
      return true;
    };
  if (inverted("default", base)) {
    return result;
  }
  for (const auto & kv : profiles) {
    if (inverted(kv.first, kv.second)) {
      return result;
    }
  }
  if (active != kDefaultProfile && profiles.find(active) == profiles.end()) {
    result.successful = false;
    result.reason = "height_profile '" + active + "' is not in height_profiles";
    return result;
  }
  base_slab_ = base;
  profiles_ = profiles;
  const bool switched = active != active_profile_;
  active_profile_ = active;
  std::tie(min_height_, max_height_) =
    active_profile_ == kDefaultProfile ? base_slab_ : profiles_.at(active_profile_);
  RCLCPP_INFO(
    this->get_logger(), "Height slab [%s]%s: [%.3f, %.3f] m", active_profile_.c_str(),
    switched ? " (switched)" : "", min_height_, max_height_);
  return result;
}

void PointCloudToLaserScanNode::footprintCallback(
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg)
{
  // Invalid or untransformable polygon clears the footprint -> fixed box only.
  std::shared_ptr<SelfFootprint> fp;
  const auto & pts = msg->polygon.points;
  if (pts.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Footprint '%s' has %zu points (< 3); ignored.", self_footprint_topic_.c_str(), pts.size());
  } else {
    try {
      // Latest TF: footprint frame -> target_frame is static (base_footprint -> base_link).
      const auto t = tf2_->lookupTransform(
        target_frame_, msg->header.frame_id, tf2::TimePointZero).transform;
      const tf2::Transform tf(
        tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
        tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));

      std::vector<tf2::Vector3> v;
      v.reserve(pts.size());
      for (const auto & p : pts) {
        v.push_back(tf * tf2::Vector3(p.x, p.y, p.z));
      }

      fp = std::make_shared<SelfFootprint>();
      fp->x_min = fp->y_min = std::numeric_limits<double>::max();
      fp->x_max = fp->y_max = std::numeric_limits<double>::lowest();
      for (size_t i = 0; i < v.size(); ++i) {
        const tf2::Vector3 & a = v[i];
        const tf2::Vector3 & b = v[(i + 1) % v.size()];
        const double ex = b.x() - a.x();
        const double ey = b.y() - a.y();
        const double len2 = ex * ex + ey * ey;
        if (len2 > 1e-12) {  // skip duplicate vertices
          fp->edges.push_back({a.x(), a.y(), ex, ey, 1.0 / len2});
        }
        fp->x_min = std::min(fp->x_min, a.x());
        fp->x_max = std::max(fp->x_max, a.x());
        fp->y_min = std::min(fp->y_min, a.y());
        fp->y_max = std::max(fp->y_max, a.y());
      }
      fp->x_min -= self_footprint_padding_;
      fp->x_max += self_footprint_padding_;
      fp->y_min -= self_footprint_padding_;
      fp->y_max += self_footprint_padding_;
      fp->received = std::chrono::steady_clock::now();
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Footprint TF %s -> %s failed: %s", msg->header.frame_id.c_str(),
        target_frame_.c_str(), ex.what());
    }
  }

  std::lock_guard<std::mutex> lock(footprint_mutex_);
  footprint_ = fp;
}

bool PointCloudToLaserScanNode::insideFootprint(
  const SelfFootprint & fp, double x, double y) const
{
  if (x < fp.x_min || x > fp.x_max || y < fp.y_min || y > fp.y_max) {
    return false;
  }
  const double pad2 = self_footprint_padding_ * self_footprint_padding_;
  bool inside = false;
  for (const auto & e : fp.edges) {
    // Even-odd crossing test (ray towards +x).
    if (((e.ay + e.ey) > y) != (e.ay > y) && x < e.ax + e.ex * (y - e.ay) / e.ey) {
      inside = !inside;
    }
    // Padding band: distance from (x, y) to this edge segment.
    const double t = std::clamp(((x - e.ax) * e.ex + (y - e.ay) * e.ey) * e.inv_len2, 0.0, 1.0);
    const double dx = x - (e.ax + t * e.ex);
    const double dy = y - (e.ay + t * e.ey);
    if (dx * dx + dy * dy < pad2) {
      return true;
    }
  }
  return inside;
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  // Prepare optional Livox `tag` iterator (kept in lock-step with x/y/z below).
  bool has_tag = false;
  for (const auto & field : cloud_msg->fields) {
    if (field.name == "tag") {
      has_tag = true;
      break;
    }
  }
  const bool apply_tag_filter = tag_filter_enable_ && has_tag;
  if (tag_filter_enable_ && !has_tag) {
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "tag_filter_enable is set but the input cloud has no 'tag' field; tag filtering disabled.");
  }
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<uint8_t>> iter_tag;
  if (apply_tag_filter) {
    iter_tag =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<uint8_t>>(*cloud_msg, "tag");
  }

  // Footprint polygon for this frame (null = fixed self box only).
  std::shared_ptr<const SelfFootprint> footprint;
  if (footprint_sub_) {
    {
      std::lock_guard<std::mutex> lock(footprint_mutex_);
      footprint = footprint_;
    }
    if (footprint && std::chrono::steady_clock::now() - footprint->received >
      std::chrono::duration<double>(self_footprint_timeout_))
    {
      footprint.reset();
    }
    const bool active = static_cast<bool>(footprint);
    if (footprint_active_.exchange(active) != active) {
      if (active) {
        RCLCPP_INFO(
          this->get_logger(), "Footprint '%s' active: self filter = fixed box UNION footprint.",
          self_footprint_topic_.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Footprint '%s' lost (stale > %.2f s or invalid): fixed self box only.",
          self_footprint_topic_.c_str(), self_footprint_timeout_);
      }
    }
  }

  // Snapshot the slab once per frame so a live change never splits a scan.
  double min_height, max_height;
  {
    std::lock_guard<std::mutex> lock(height_mutex_);
    min_height = min_height_;
    max_height = max_height_;
  }

  std::unique_ptr<GhostFilter> ghost;
  if (ghost_filter_enable_) {
    ghost = std::make_unique<GhostFilter>(ghost_params_);
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // Advance the tag iterator every iteration so it stays aligned with x/y/z.
    uint8_t tag_val = 0;
    if (iter_tag) {
      tag_val = **iter_tag;
      ++(*iter_tag);
    }

    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    // Drop Livox noise points (dust/rain/fog/blooming) flagged by the tag field.
    if (apply_tag_filter && (tag_val & static_cast<uint8_t>(tag_filter_mask_)) != 0) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for tag 0x%02x (mask 0x%02x)\n", tag_val, tag_filter_mask_);
      continue;
    }

    if (*iter_z > max_height || *iter_z < min_height) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height, max_height);
      continue;
    }

    // Ghost-filter neighbor pool: every slab point, including self-box and out-of-range ones.
    if (ghost) {
      ghost->addPoolPoint(*iter_x, *iter_y, *iter_z);
    }

    // Self-filter on x/y: drop points INSIDE the body box, or (union) inside the padded footprint.
    // The box is checked first so body points never reach the polygon test.
    if (use_self_filter_ &&
      ((*iter_x >= self_x_min_ && *iter_x <= self_x_max_ &&
      *iter_y >= self_y_min_ && *iter_y <= self_y_max_) ||
      (footprint && insideFootprint(*footprint, *iter_x, *iter_y))))
    {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for x/y (%f, %f) inside self box ([%f,%f],[%f,%f]) or footprint\n",
        *iter_x, *iter_y, self_x_min_, self_x_max_, self_y_min_, self_y_max_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (ghost) {
      ghost->addScanPoint(*iter_x, *iter_y, *iter_z, index, range);
    }
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }

  if (ghost) {
    const std::size_t changed = ghost->apply(scan_msg->ranges);
    RCLCPP_DEBUG(this->get_logger(), "ghost filter moved %zu bins", changed);
  }
  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
