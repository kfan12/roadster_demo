// stunt_mission_node.cpp - roadster stunt mission FSM + ground and flight control.
//
// Mode "ramp" (Demo A, the inverted ramp):
//   ACCELERATE     fan on, speed up along the run-in toward the ramp
//   CLIMB          round the quarter-pipe arc at climb_speed, held on by the fan
//   STOP           brake (starting on the upper arc) to a stop at hold_x, just past the top
//   HOLD_INVERTED  stationary, upside down, fan only, for hold_inverted_time
//   DRIVE_OFF      from standstill, still upside down, accelerate forward (wheels
//                  plus the rear nozzles) to drive_off_speed and let go near the
//                  overhang's end
//   ...then the flight sequence below, flipping with that forward speed
//   (or stay held if release_after_hold is false)
// Mode "drop" (the flight half on its own): the car starts upside down under a
// ceiling held by its fan, the same state as HOLD_INVERTED:
//   HOLD           fan on, wait hold_time
// Flight sequence (both modes):
//   RELEASE        fan off, hold the start attitude while the car falls clear
//   FLIP           track a minimum-jerk rotation of flip_turns turns about the
//                  body flip_axis (0.5 turn: inverted -> upright, plan correction 1)
//   DESCENT        hold the final attitude, free fall
//   LAND_BRAKE     once descending faster than the constant-deceleration profile,
//                  burn to touch down at touchdown_speed
//   LANDED         base_link on the ground and upright: thrusters and fan off
//   ROLLOUT        brake the touchdown speed away at stop_decel, keep straight
//   STOPPED        standing on the wheels
//   ABORT          mission timeout or tank empty while airborne
//
// The node is the only publisher of thruster / fan / wheel / steering commands
// while it runs (plan section 4, command authority). State comes from
// /odom_truth (ground truth first, plan 5.10); velocities are taken by finite
// differences of the 200 Hz poses, which is frame-unambiguous. Ramp geometry
// comes from the world JSON (maps/<world>.json, one owner: the generator).
#include <yaml-cpp/yaml.h>

#include <av_feature_msgs/msg/stunt_status.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/float32_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "roadster_control/allocator.hpp"
#include "roadster_control/attitude.hpp"
#include "roadster_control/landing.hpp"

namespace roadster_control
{

namespace
{
Vec3 vec3Param(const std::vector<double> & v)
{
  return Vec3(v.at(0), v.at(1), v.at(2));
}

constexpr double kG = 9.81;
const std::array<const char *, 4> kWheelShort{"fl", "fr", "rl", "rr"};
const std::array<const char *, 4> kWheelJoint{
  "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint",
  "rear_right_wheel_joint"};
}  // namespace

class StuntMissionNode : public rclcpp::Node
{
public:
  StuntMissionNode()
  : Node("stunt_mission_node")
  {
    mode_ = declare_parameter<std::string>("mode", "drop");
    mass_ = declare_parameter<double>("mass", 1800.01);
    inertia_ = vec3Param(declare_parameter<std::vector<double>>(
        "inertia_diag", {584.5, 3119.5, 3562.0})).asDiagonal();
    com_chassis_ = vec3Param(declare_parameter<std::vector<double>>(
        "com_chassis", {0.0075, 0.0, 0.0}));
    chassis_z_ = declare_parameter<double>("chassis_offset_z", 0.33);
    landed_com_height_ = declare_parameter<double>("landed_com_height", 0.33);
    // flight
    hold_time_ = declare_parameter<double>("hold_time", 1.0);
    flip_axis_ = vec3Param(declare_parameter<std::vector<double>>(
        "flip_axis", {1.0, 0.0, 0.0})).normalized();
    flip_turns_ = declare_parameter<double>("flip_turns", 0.5);
    flip_delay_ = declare_parameter<double>("flip_delay", 0.4);
    flip_duration_ = declare_parameter<double>("flip_duration", 0.8);
    gains_.kp = vec3Param(declare_parameter<std::vector<double>>("att_kp", {36.0, 36.0, 36.0}));
    gains_.kd = vec3Param(declare_parameter<std::vector<double>>("att_kd", {10.8, 10.8, 10.8}));
    LandingParams lp;
    lp.brake_accel = declare_parameter<double>("brake_accel", lp.brake_accel);
    lp.touchdown_speed = declare_parameter<double>("touchdown_speed", lp.touchdown_speed);
    lp.k_v = declare_parameter<double>("brake_kv", lp.k_v);
    landing_ = std::make_unique<LandingBrake>(lp);
    landed_load_ = declare_parameter<double>("landed_load", 0.1);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_truth");
    mission_timeout_ = declare_parameter<double>("mission_timeout", 40.0);
    start_delay_ = declare_parameter<double>("start_delay", 0.0);
    lambda_ = declare_parameter<double>("alloc_lambda", 1e-4);
    w_flight_ = vec6(declare_parameter<std::vector<double>>(
        "alloc_weights_flight", {0.01, 0.01, 0.01, 1.0, 1.0, 1.0}));
    w_brake_ = vec6(declare_parameter<std::vector<double>>(
        "alloc_weights_brake", {0.05, 0.05, 1.0, 1.0, 1.0, 1.0}));
    // ground (ramp mode)
    climb_speed_ = declare_parameter<double>("climb_speed", 6.0);
    accel_ = declare_parameter<double>("accel", 2.0);
    stop_decel_ = declare_parameter<double>("stop_decel", 3.0);
    hold_x_ = declare_parameter<double>("hold_x", -1.5);
    hold_kx_ = declare_parameter<double>("hold_kx", 1.0);
    hold_inverted_time_ = declare_parameter<double>("hold_inverted_time", 2.0);
    release_after_hold_ = declare_parameter<bool>("release_after_hold", true);
    drive_off_speed_ = declare_parameter<double>("drive_off_speed", 7.0);
    drive_off_accel_ = declare_parameter<double>("drive_off_accel", 2.5);
    drive_off_flip_delay_ = declare_parameter<double>("drive_off_flip_delay", 0.6);
    release_margin_ = declare_parameter<double>("release_margin", 0.5);
    wheelbase_ = declare_parameter<double>("wheelbase", 2.7);
    speed_kp_ = declare_parameter<double>("speed_kp", 2.0);
    speed_ki_ = declare_parameter<double>("speed_ki", 0.5);
    tc_k_ = declare_parameter<double>("traction_k", 0.8);
    tire_mu_ = declare_parameter<double>("tire_mu", 1.2);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.33);
    steer_k_lat_ = declare_parameter<double>("steer_k_lat", 0.3);
    steer_k_head_ = declare_parameter<double>("steer_k_head", 1.0);
    steer_max_ = declare_parameter<double>("steer_max", 0.3);

    loadNozzles(declare_parameter<std::string>("nozzles_file", ""));
    const std::string world_json = declare_parameter<std::string>("world_json", "");
    if (mode_ == "ramp") {
      loadRamp(world_json);
    }

    pub_thr_ = create_publisher<ros_gz_interfaces::msg::Float32Array>("/roadster/thruster_cmd", 10);
    pub_fan_ = create_publisher<std_msgs::msg::Float64>("/roadster/fan_cmd", 10);
    pub_wheel_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wheel_effort_controller/commands", 10);
    pub_steer_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/steering_position_controller/commands", 10);
    pub_status_ = create_publisher<av_feature_msgs::msg::StuntStatus>("/stunt/status", 10);
    sub_feed_ = create_subscription<std_msgs::msg::Float64>(
      "/roadster/tank/feed_ratio", 10, [this](std_msgs::msg::Float64::SharedPtr m) {
        feed_ratio_ = m->data;
      });
    sub_mass_ = create_subscription<std_msgs::msg::Float64>(
      "/roadster/tank/mass", 10, [this](std_msgs::msg::Float64::SharedPtr m) {
        tank_mass_ = m->data;
      });
    sub_thrust_ = create_subscription<ros_gz_interfaces::msg::Float32Array>(
      "/roadster/thruster_state", 50, [this](ros_gz_interfaces::msg::Float32Array::SharedPtr m) {
        onThrust(*m);
      });
    for (std::size_t i = 0; i < 4; ++i) {
      sub_wrench_[i] = create_subscription<geometry_msgs::msg::Wrench>(
        std::string("/roadster/wheel_wrench/") + kWheelShort[i], 50,
        [this, i](geometry_msgs::msg::Wrench::SharedPtr m) {
          wheel_force_[i] = Vec3(m->force.x, m->force.y, m->force.z);
        });
    }
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50, [this](sensor_msgs::msg::JointState::SharedPtr m) {
        for (std::size_t k = 0; k < m->name.size() && k < m->position.size(); ++k) {
          for (std::size_t i = 0; i < 4; ++i) {
            if (m->name[k] == kWheelJoint[i]) {
              wheel_angle_[i] = m->position[k];
            }
          }
        }
      });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50, [this](nav_msgs::msg::Odometry::SharedPtr m) {onOdom(*m);});
    RCLCPP_INFO(get_logger(), "mode '%s', %zu nozzles, mass %.1f kg, flip %.2f turns%s",
      mode_.c_str(), alloc_->size(), mass_, flip_turns_,
      mode_ == "ramp" ? (", ramp R = " + std::to_string(ramp_r_) + " m").c_str() : "");
  }

private:
  static Vec6 vec6(const std::vector<double> & v)
  {
    Vec6 r;
    for (int i = 0; i < 6; ++i) {
      r(i) = v.at(i);
    }
    return r;
  }

  void loadNozzles(const std::string & path)
  {
    const YAML::Node root = YAML::LoadFile(path)["nozzles"];
    std::map<std::string, YAML::Node> sorted;  // the actuation system's order
    for (const auto & kv : root) {
      sorted[kv.first.as<std::string>()] = kv.second;
    }
    std::vector<NozzleGeom> geo;
    for (const auto & [name, n] : sorted) {
      NozzleGeom g;
      g.name = name;
      g.r_com = vec3Param(n["xyz"].as<std::vector<double>>()) - com_chassis_;
      g.dir = vec3Param(n["force_dir"].as<std::vector<double>>()).normalized();
      g.f_max = n["f_max_mg"].as<double>() * mass_ * kG;
      if (name.rfind("rear_main", 0) == 0) {
        rear_idx_.push_back(static_cast<int>(geo.size()));
        rear_f_max_ += g.f_max;
      }
      geo.push_back(g);
    }
    alloc_ = std::make_unique<ThrustAllocator>(geo, mass_ * kG, mass_ * kG * 1.0);
  }

  void loadRamp(const std::string & path)
  {
    std::ifstream f(path);
    if (!f) {
      throw std::runtime_error("ramp mode needs world_json, cannot open '" + path + "'");
    }
    const auto j = nlohmann::json::parse(f).at("geometry");
    if (j.at("kind") != "ramp") {
      throw std::runtime_error(path + " is not a ramp world");
    }
    ramp_r_ = j.at("radius");
    ramp_cx_ = j.at("arc_center_xz")[0];
    ramp_cz_ = j.at("arc_center_xz")[1];
    arc_len_ = ramp_r_ * (static_cast<double>(j.at("arc_theta")[1]) -
      static_cast<double>(j.at("arc_theta")[0]));
    const double overhang_end = ramp_cx_ - static_cast<double>(j.at("overhang_len"));
    // let go while the front axle (wheelbase / 2 ahead of base_link, in -x)
    // is still release_margin short of the edge
    const double release_x = overhang_end + wheelbase_ / 2.0 + release_margin_;
    s_hold_ = arc_len_ + (ramp_cx_ - hold_x_);
    s_release_ = arc_len_ + (ramp_cx_ - release_x);
    if (hold_x_ - wheelbase_ / 2.0 < release_x - 0.05 || hold_x_ + wheelbase_ / 2.0 > ramp_cx_) {
      RCLCPP_WARN(get_logger(), "hold_x %.2f does not put both axles on the overhang "
        "[%.2f, %.2f]", hold_x_, overhang_end, ramp_cx_);
    }
    RCLCPP_INFO(get_logger(), "hold at s = %.2f m, let go at s = %.2f m (%.2f m of run-up)",
      s_hold_, s_release_, s_release_ - s_hold_);
  }

  // distance along the track at base_link (which sits on the driving surface):
  // run-in (< 0), the arc (0 .. arc length), then the overhang
  double pathS(const Vec3 & p) const
  {
    const double dx = p.x() - ramp_cx_, dz = p.z() - ramp_cz_;
    if (dz < 0.0 && dx < 0.0) {
      return dx;
    }
    if (dz >= 0.0 && dx < 0.0) {
      return arc_len_ - dx;
    }
    return std::clamp(ramp_r_ * (std::atan2(dz, dx) + M_PI / 2), 0.0, arc_len_);
  }

  // world-up force all nozzles could make at the current attitude (feed 1)
  double upCapacity(const Eigen::Matrix3d & R) const
  {
    double c = 0.0;
    for (const auto & n : alloc_->nozzles()) {
      c += n.f_max * std::max(0.0, (R * n.dir).z());
    }
    return c;
  }

  // wheel normal force along chassis +z from the F/T reading (spinning wheel
  // frame -> chassis frame by the joint angle; sign checked on the flat world)
  double wheelNormal(std::size_t i) const
  {
    const double a = wheel_angle_[i];
    const Vec3 & f = wheel_force_[i];
    return -std::sin(a) * f.x() + std::cos(a) * f.z();
  }

  void onThrust(const ros_gz_interfaces::msg::Float32Array & m)
  {
    const double t = now().seconds();
    double sum = 0;
    for (float f : m.data) {
      sum += f;
    }
    if (mission_started_ && last_thrust_t_ > 0 && t > last_thrust_t_) {
      impulse_ += sum * (t - last_thrust_t_);
    }
    last_thrust_t_ = t;
  }

  void enter(const std::string & s, double t)
  {
    RCLCPP_INFO(get_logger(), "[%.3f] %s -> %s", t, state_.c_str(), s.c_str());
    state_ = s;
    t_state0_ = t;
    speed_int_ = 0.0;
  }

  bool flying() const
  {
    return state_ == "RELEASE" || state_ == "FLIP" || state_ == "DESCENT" ||
           state_ == "LAND_BRAKE";
  }

  void startFlight(const Quat & q, double t, std::optional<double> delay = std::nullopt)
  {
    flip_delay_now_ = delay.value_or(flip_delay_);
    flip_ = std::make_unique<FlipReference>(
      q, flip_axis_, 2.0 * M_PI * flip_turns_, t + flip_delay_now_, flip_duration_);
    roll_progress_ = 0.0;
    enter("RELEASE", t);
  }

  void onOdom(const nav_msgs::msg::Odometry & m)
  {
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const auto & pp = m.pose.pose.position;
    const auto & o = m.pose.pose.orientation;
    const Vec3 p(pp.x, pp.y, pp.z);
    const Quat q(o.w, o.x, o.y, o.z);
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const Vec3 com = p + R * (com_chassis_ + Vec3(0, 0, chassis_z_));

    if (!prev_ || t <= prev_->t) {
      prev_ = Prev{t, com, p, q};
      return;
    }
    const double dt = t - prev_->t;
    const double vz = (com.z() - prev_->com.z()) / dt;
    const Vec3 w = attitudeError(prev_->q, q) / dt;  // body rate
    const Vec3 v_world = (p - prev_->p) / dt;
    speed_ += 0.2 * ((R.transpose() * v_world).x() - speed_);  // ~25 ms EMA at 200 Hz
    roll_progress_ += w.dot(flip_axis_) * dt;
    prev_ = Prev{t, com, p, q};
    const double h = com.z() - landed_com_height_;

    if (!feed_ratio_) {
      return;  // actuation state not seen yet
    }
    if (!first_t_) {
      first_t_ = t;
    }
    if (!mission_started_ && t - *first_t_ < start_delay_) {
      return;  // let ros2_control and the viewers come up first
    }
    if (!mission_started_) {
      mission_started_ = true;
      t_mission0_ = t;
      enter(mode_ == "ramp" ? "ACCELERATE" : "HOLD", t);
    }
    const double ts = t - t_state0_;
    const double tm = t - t_mission0_;
    if (state_ != "ROLLOUT" && state_ != "STOPPED" && state_ != "ABORT" &&
      tm > mission_timeout_)
    {
      enter("ABORT", t);
    }

    double fan = 0.0;
    double f_up = 0.0;
    Vec3 tau = Vec3::Zero();
    double att_err = 0.0;
    Vec6 weights = w_flight_;
    bool drive = false;
    double v_ref = 0.0;
    double a_ff = 0.0;  // reference acceleration along body x [m/s^2]
    const double arc_angle = std::atan2(p.z() - ramp_cz_, p.x() - ramp_cx_) + M_PI / 2;

    // ---------------- ground phase (ramp mode) ----------------
    const double s = pathS(p);  // distance along the track (run-in < 0 < arc < overhang)
    if (state_ == "ACCELERATE") {
      fan = 1.0;
      drive = true;
      v_ref = std::min(climb_speed_, accel_ * ts);
      if (s >= 0.0) {
        enter("CLIMB", t);
      }
    } else if (state_ == "CLIMB") {
      fan = 1.0;
      drive = true;
      v_ref = climb_speed_;
      // the overhang is short, so braking starts on the upper arc as soon as
      // the remaining track equals the stopping distance
      const double v = std::max(speed_, 0.1);
      if (s_hold_ - s <= v * v / (2.0 * stop_decel_)) {
        enter("STOP", t);
      }
    } else if (state_ == "STOP" || state_ == "HOLD_INVERTED") {
      // Stop at s_hold on a constant-deceleration trajectory planned at entry
      // (track its position and speed, with its deceleration as feed-forward),
      // then hold the point with a position loop. A speed-vs-distance law
      // like min(sqrt(2 a d), k d) demands 2a at its crossover - more than
      // the traction there - so the trajectory is planned once instead.
      fan = 1.0;
      drive = true;
      const double dist = s_hold_ - s;  // track still to go
      if (state_ == "STOP") {
        if (!stop_plan_) {
          const double v0 = std::max(speed_, 0.1);
          const double a = v0 * v0 / (2.0 * std::max(dist, 0.1));
          if (a > 1.2 * stop_decel_) {
            RCLCPP_WARN(get_logger(), "stopping at hold_x needs %.2f m/s^2 > stop_decel %.2f",
              a, stop_decel_);
          }
          stop_plan_ = StopPlan{t, v0, a, s};
        }
        const auto & sp = *stop_plan_;
        const double tt = std::min(t - sp.t0, sp.v0 / sp.a);
        const double s_ref = sp.v0 * tt - 0.5 * sp.a * tt * tt;
        const double e_s = s_ref - (s - sp.s0);  // > 0: behind the plan
        v_ref = std::max(sp.v0 - sp.a * tt, 0.0) + hold_kx_ * e_s;
        a_ff = t - sp.t0 < sp.v0 / sp.a ? -sp.a : 0.0;
        if (std::abs(dist) < 0.01 && std::abs(speed_) < 0.02) {
          enter("HOLD_INVERTED", t);
        }
      } else {
        v_ref = hold_kx_ * dist;
        if (ts >= hold_inverted_time_ && release_after_hold_) {
          enter("DRIVE_OFF", t);
        }
      }
    } else if (state_ == "DRIVE_OFF") {
      // From standstill, still upside down on the fan: accelerate forward and
      // let go while the front axle is still under the overhang, so all four
      // wheels leave together and the car flies on with this speed. The short
      // overhang leaves too little run for the tyres alone, so the two rear
      // nozzles supply the rest (Tesla's thrusters "launched it off the ramp");
      // the wheels track the speed trajectory, the thrust is feed-forward.
      fan = 1.0;
      drive = true;
      const double run = std::max(s_release_ - s_hold_, 0.1);
      const double a_total = drive_off_speed_ * drive_off_speed_ / (2.0 * run);
      v_ref = std::min(drive_off_speed_, a_total * ts);
      a_ff = v_ref < drive_off_speed_ ? a_total : 0.0;
      const double a_thrust = std::max(a_total - drive_off_accel_, 0.0);
      launch_u_ = v_ref < drive_off_speed_ && rear_f_max_ > 0 ?
        std::clamp(mass_ * a_thrust / (rear_f_max_ * *feed_ratio_), 0.0, 1.0) : 0.0;
      if (s >= s_release_) {
        release_speed_ = speed_;
        launch_u_ = 0.0;
        startFlight(q, t, drive_off_flip_delay_);
      }
    } else if (state_ == "LANDED" || state_ == "ROLLOUT" || state_ == "STOPPED") {
      // on the wheels again: brake the touchdown speed away, keep straight
      drive = true;
      if (state_ == "LANDED") {
        rollout_v0_ = std::max(speed_, 0.0);
        enter("ROLLOUT", t);
      }
      if (state_ == "ROLLOUT") {
        const double tt = t - t_state0_;
        v_ref = std::max(rollout_v0_ - stop_decel_ * tt, 0.0);
        a_ff = v_ref > 0.0 ? -stop_decel_ : 0.0;
        if (v_ref <= 0.0) {
          // braking profile done: hold this spot (a speed-only loop let the
          // integral overshoot and creep backwards at 0.2 m/s)
          stop_point_ = p;
          enter("STOPPED", t);
        }
      }
      if (state_ == "STOPPED") {
        v_ref = hold_kx_ * (stop_point_ - p).dot(R.col(0));  // forward distance to it
      }
    } else if (state_ == "HOLD") {
      fan = 1.0;
      if (ts >= hold_time_) {
        startFlight(q, t);
      }
    }

    // ---------------- flight ----------------
    if (flying()) {
      if (state_ == "RELEASE" && t >= t_state0_ + flip_delay_now_) {
        enter("FLIP", t);
      }
      if (state_ == "FLIP" && t >= flip_->endTime()) {
        enter("DESCENT", t);
      }
      const auto ref = flip_->at(t);
      tau = attitudeTorque(inertia_, gains_, q, w, ref.q, ref.w, ref.alpha);
      att_err = attitudeError(ref.q, q).norm();
      f_up = landing_->upForce(h, vz, mass_, 0.9 * upCapacity(R) * *feed_ratio_);
      if (landing_->active() && state_ != "LAND_BRAKE") {
        enter("LAND_BRAKE", t);
      }
      if (landing_->active()) {
        weights = w_brake_;
      }
      // touchdown = wheel contact while upright (force/torque sensors; an
      // airborne wheel reads negative), with a low estimated height as the
      // backup. Contact, not "carrying the car's weight": the brake burn still
      // holds ~2.5 m g at touchdown, so a 0.5 m g load threshold was missed,
      // the car bounced and hovered on thrust (est_shadow3).
      double load = 0.0;
      for (std::size_t i = 0; i < 4; ++i) {
        load += std::max(wheelNormal(i), 0.0);
      }
      const bool upright = R.col(2).z() > std::cos(0.35);
      if (upright && state_ == "LAND_BRAKE" &&
        (load > landed_load_ * mass_ * kG || h < 0.15))
      {
        enter("LANDED", t);
      } else if (*feed_ratio_ <= 0.0) {
        enter("ABORT", t);
      }
    }

    // ---------------- ground drive: speed PI + traction control + steering ----
    std::array<double, 4> wheel_tau{0, 0, 0, 0};
    double steer = 0.0, lat = 0.0, n_min = 1e9;
    for (std::size_t i = 0; i < 4; ++i) {
      n_min = std::min(n_min, wheelNormal(i));
    }
    if (drive) {
      const double g_along = (R.transpose() * Vec3(0, 0, -kG)).x();  // gravity along body x
      const double e = v_ref - speed_;
      speed_int_ = std::clamp(speed_int_ + e * dt, -2.0, 2.0);
      const double f = mass_ * (a_ff + speed_kp_ * e + speed_ki_ * speed_int_ - g_along);
      for (std::size_t i = 0; i < 4; ++i) {
        const double cap = tc_k_ * tire_mu_ * std::max(wheelNormal(i), 0.0) * wheel_radius_;
        wheel_tau[i] = std::clamp(f / 4.0 * wheel_radius_, -cap, cap);
      }
      // lateral: the ramp's width axis is world y; "left" flips with body y
      const double side = R.col(1).y() >= 0 ? 1.0 : -1.0;
      lat = p.y();
      const double head = std::asin(std::clamp(R.col(0).y() * side, -1.0, 1.0));
      steer = std::clamp(-steer_k_lat_ * lat * side - steer_k_head_ * head, -steer_max_, steer_max_);
    }

    // ---------------- allocate + publish ----------------
    Vec6 w_des = Vec6::Zero();
    AllocResult a;
    a.u = Eigen::VectorXd::Zero(alloc_->size());
    a.achieved.setZero();
    if (flying() && state_ != "LANDED") {
      // forces requested and weighted in WORLD axes (lift up, no horizontal
      // push whatever the attitude), torques in body axes
      w_des.head<3>() = Vec3(0, 0, f_up);
      w_des.tail<3>() = tau;
      a = alloc_->solve(w_des, weights, *feed_ratio_, lambda_, R);
      a.achieved.head<3>() = R * a.achieved.head<3>();  // report in the same axes
    }
    if (state_ == "DRIVE_OFF") {
      for (int i : rear_idx_) {
        a.u(i) = launch_u_;  // rear thrust assist for the launch off the overhang
      }
    }
    ros_gz_interfaces::msg::Float32Array cmd;
    for (int i = 0; i < a.u.size(); ++i) {
      cmd.data.push_back(static_cast<float>(a.u(i)));
    }
    pub_thr_->publish(cmd);
    std_msgs::msg::Float64 fm;
    fm.data = fan;
    pub_fan_->publish(fm);
    std_msgs::msg::Float64MultiArray wm;
    wm.data.assign(wheel_tau.begin(), wheel_tau.end());
    pub_wheel_->publish(wm);
    wm.data = {steer, steer};
    pub_steer_->publish(wm);

    av_feature_msgs::msg::StuntStatus st;
    st.header = m.header;
    st.state = state_;
    st.t_state = t - t_state0_;
    st.t_mission = tm;
    st.height = h;
    st.vz = vz;
    st.roll_progress = roll_progress_;
    st.attitude_error = att_err;
    st.speed = speed_;
    st.speed_ref = v_ref;
    st.arc_angle = mode_ == "ramp" ? arc_angle : 0.0;
    st.lateral_offset = lat;
    st.min_wheel_normal = n_min;
    for (std::size_t i = 0; i < 4; ++i) {
      st.wheel_torque[i] = wheel_tau[i];
    }
    st.tank_mass = tank_mass_;
    st.feed_ratio = *feed_ratio_;
    st.impulse_used = impulse_;
    st.fan_cmd = fan;
    for (int i = 0; i < 6; ++i) {
      st.wrench_des[i] = w_des(i);
      st.wrench_alloc[i] = a.achieved(i);
    }
    pub_status_->publish(st);
  }

  struct Prev
  {
    double t;
    Vec3 com, p;
    Quat q;
  };
  struct StopPlan
  {
    double t0, v0, a, s0;
  };
  std::optional<StopPlan> stop_plan_;

  std::string mode_;
  double mass_;
  Mat3 inertia_;
  Vec3 com_chassis_;
  double chassis_z_, landed_com_height_;
  double hold_time_;
  Vec3 flip_axis_;
  double flip_turns_, flip_delay_, flip_duration_;
  AttitudeGains gains_;
  std::unique_ptr<LandingBrake> landing_;
  double landed_load_, mission_timeout_, start_delay_;
  std::string odom_topic_;
  std::optional<double> first_t_;
  double lambda_;
  Vec6 w_flight_, w_brake_;
  std::unique_ptr<ThrustAllocator> alloc_;
  double climb_speed_, accel_, stop_decel_, hold_x_, hold_kx_, hold_inverted_time_;
  bool release_after_hold_;
  double drive_off_speed_, drive_off_accel_, drive_off_flip_delay_, release_margin_, wheelbase_;
  double release_speed_ = 0.0, rollout_v0_ = 0.0, flip_delay_now_ = 0.0;
  double arc_len_ = 0.0, s_hold_ = 0.0, s_release_ = 0.0;
  std::vector<int> rear_idx_;   // rear main nozzles, in allocator / command order
  double rear_f_max_ = 0.0;     // their combined f_max at feed 1 [N]
  double launch_u_ = 0.0;       // rear valve command during DRIVE_OFF
  Vec3 stop_point_ = Vec3::Zero();
  double speed_kp_, speed_ki_, tc_k_, tire_mu_, wheel_radius_;
  double steer_k_lat_, steer_k_head_, steer_max_;
  double ramp_r_ = 0.0, ramp_cx_ = 0.0, ramp_cz_ = 0.0;

  std::optional<double> feed_ratio_;
  double tank_mass_ = 0.0;
  double impulse_ = 0.0;
  double last_thrust_t_ = -1.0;
  std::optional<Prev> prev_;
  double speed_ = 0.0, speed_int_ = 0.0;
  std::array<Vec3, 4> wheel_force_{};
  std::array<double, 4> wheel_angle_{0, 0, 0, 0};
  double roll_progress_ = 0.0;
  bool mission_started_ = false;
  double t_mission0_ = 0.0, t_state0_ = 0.0;
  std::string state_ = "WAIT";
  std::unique_ptr<FlipReference> flip_;

  rclcpp::Publisher<ros_gz_interfaces::msg::Float32Array>::SharedPtr pub_thr_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fan_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheel_, pub_steer_;
  rclcpp::Publisher<av_feature_msgs::msg::StuntStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_feed_, sub_mass_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Float32Array>::SharedPtr sub_thrust_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr, 4> sub_wrench_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

}  // namespace roadster_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roadster_control::StuntMissionNode>());
  rclcpp::shutdown();
  return 0;
}
