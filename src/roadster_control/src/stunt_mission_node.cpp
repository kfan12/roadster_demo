// stunt_mission_node.cpp - roadster stunt mission FSM + flight control.
//
// Mode "drop" (plan step 3, the flip/landing half of Demo A on its own): the
// car starts upside down under a ceiling held by its suction fan, exactly the
// state at the top of the inverted ramp. Sequence:
//   HOLD      fan on, wait hold_time
//   RELEASE   fan off, hold the start attitude while the car falls clear
//   FLIP      track a minimum-jerk rotation of flip_turns turns about the body
//             flip_axis (0.5 turn: inverted -> upright, plan correction 1)
//   DESCENT   hold the final attitude, free fall
//   LAND_BRAKE once the car descends faster than the constant-deceleration
//             profile, burn the down nozzles to touch down at touchdown_speed
//   LANDED    base_link at the ground and upright: everything off
//   ABORT     mission timeout or tank empty while airborne
//
// The node is the only publisher of thruster / fan / wheel commands while it
// runs (plan section 4, command authority). State comes from /odom_truth
// (ground truth first, plan 5.10); velocities are taken by finite differences
// of the 200 Hz poses, which is frame-unambiguous.
#include <yaml-cpp/yaml.h>

#include <av_feature_msgs/msg/stunt_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/float32_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <cmath>
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

// body-frame angular velocity taking R0 to R1 in dt
Vec3 bodyRate(const Quat & q0, const Quat & q1, double dt)
{
  return attitudeError(q0, q1) / dt;
}
}  // namespace

class StuntMissionNode : public rclcpp::Node
{
public:
  StuntMissionNode()
  : Node("stunt_mission_node")
  {
    mode_ = declare_parameter<std::string>("mode", "drop");
    mass_ = declare_parameter<double>("mass", 1800.01);
    const auto idiag = declare_parameter<std::vector<double>>(
      "inertia_diag", {584.5, 3119.5, 3562.0});
    inertia_ = vec3Param(idiag).asDiagonal();
    com_chassis_ = vec3Param(declare_parameter<std::vector<double>>(
        "com_chassis", {0.0075, 0.0, 0.0}));
    chassis_z_ = declare_parameter<double>("chassis_offset_z", 0.33);
    landed_com_height_ = declare_parameter<double>("landed_com_height", 0.33);
    hold_time_ = declare_parameter<double>("hold_time", 1.0);
    flip_axis_ = vec3Param(declare_parameter<std::vector<double>>("flip_axis", {1.0, 0.0, 0.0}));
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
    landed_z_ = declare_parameter<double>("landed_z", 0.03);
    mission_timeout_ = declare_parameter<double>("mission_timeout", 15.0);
    lambda_ = declare_parameter<double>("alloc_lambda", 1e-4);
    w_flight_ = vec6(declare_parameter<std::vector<double>>(
        "alloc_weights_flight", {0.01, 0.01, 0.01, 1.0, 1.0, 1.0}));
    w_brake_ = vec6(declare_parameter<std::vector<double>>(
        "alloc_weights_brake", {0.05, 0.05, 1.0, 1.0, 1.0, 1.0}));
    loadNozzles(declare_parameter<std::string>("nozzles_file", ""));

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
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom_truth", 50, [this](nav_msgs::msg::Odometry::SharedPtr m) {onOdom(*m);});
    RCLCPP_INFO(get_logger(), "mode '%s', %zu nozzles, mass %.1f kg, flip %.2f turns",
      mode_.c_str(), alloc_ ? alloc_->size() : 0, mass_, flip_turns_);
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
      g.f_max = n["f_max_mg"].as<double>() * mass_ * 9.81;
      geo.push_back(g);
    }
    alloc_ = std::make_unique<ThrustAllocator>(geo, mass_ * 9.81, mass_ * 9.81 * 1.0);
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
  }

  void onOdom(const nav_msgs::msg::Odometry & m)
  {
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const auto & p = m.pose.pose.position;
    const auto & o = m.pose.pose.orientation;
    const Quat q(o.w, o.x, o.y, o.z);
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const Vec3 com = Vec3(p.x, p.y, p.z) + R * (com_chassis_ + Vec3(0, 0, chassis_z_));

    if (!prev_ || t <= prev_->t) {
      prev_ = Prev{t, com, q};
      return;
    }
    const double dt = t - prev_->t;
    const double vz = (com.z() - prev_->com.z()) / dt;
    const Vec3 w = bodyRate(prev_->q, q, dt);
    roll_progress_ += w.dot(flip_axis_.normalized()) * dt * (mission_started_ ? 1 : 0);
    prev_ = Prev{t, com, q};
    const double h = com.z() - landed_com_height_;

    if (!feed_ratio_) {
      return;  // actuation state not seen yet
    }
    if (!mission_started_) {
      mission_started_ = true;
      t_mission0_ = t;
      enter("HOLD", t);
    }
    const double ts = t - t_state0_;
    const double tm = t - t_mission0_;

    double fan = 0.0;
    double f_up = 0.0;
    Vec3 tau = Vec3::Zero();
    bool control = false;
    double att_err = 0.0;
    Vec6 weights = w_flight_;

    if (state_ != "LANDED" && state_ != "ABORT" && tm > mission_timeout_) {
      enter("ABORT", t);
    }
    if (state_ == "HOLD") {
      fan = 1.0;
      if (ts >= hold_time_) {
        flip_ = std::make_unique<FlipReference>(
          q, flip_axis_, 2.0 * M_PI * flip_turns_, t + flip_delay_, flip_duration_);
        roll_progress_ = 0.0;
        enter("RELEASE", t);
      }
    }
    if (state_ == "RELEASE" || state_ == "FLIP" || state_ == "DESCENT" ||
      state_ == "LAND_BRAKE")
    {
      control = true;
      if (state_ == "RELEASE" && t >= t_state0_ + flip_delay_) {
        enter("FLIP", t);
      }
      if (state_ == "FLIP" && t >= flip_->endTime()) {
        enter("DESCENT", t);
      }
      const auto ref = flip_->at(t);
      tau = attitudeTorque(inertia_, gains_, q, w, ref.q, ref.w, ref.alpha);
      att_err = attitudeError(ref.q, q).norm();
      const double up = R.col(2).z();  // how upright the body z axis is
      f_up = landing_->upForce(h, vz, mass_, 0.9 * upCapacity(R) * *feed_ratio_);
      if (landing_->active() && state_ != "LAND_BRAKE") {
        enter("LAND_BRAKE", t);
      }
      if (landing_->active()) {
        weights = w_brake_;
      }
      if (p.z < landed_z_ && up > std::cos(0.35)) {
        enter("LANDED", t);
        control = false;
      }
      if (*feed_ratio_ <= 0.0 && state_ != "LANDED") {
        enter("ABORT", t);
        control = false;
      }
    }

    Vec6 w_des = Vec6::Zero();
    AllocResult a;
    a.u = Eigen::VectorXd::Zero(alloc_->size());
    a.achieved.setZero();
    if (control) {
      // forces requested and weighted in WORLD axes (lift up, no horizontal
      // push whatever the attitude), torques in body axes
      w_des.head<3>() = Vec3(0, 0, f_up);
      w_des.tail<3>() = tau;
      a = alloc_->solve(w_des, weights, *feed_ratio_, lambda_, R);
      a.achieved.head<3>() = R * a.achieved.head<3>();  // report in the same axes
    }

    ros_gz_interfaces::msg::Float32Array cmd;
    for (int i = 0; i < a.u.size(); ++i) {
      cmd.data.push_back(static_cast<float>(a.u(i)));
    }
    pub_thr_->publish(cmd);
    std_msgs::msg::Float64 fm;
    fm.data = fan;
    pub_fan_->publish(fm);
    if (++tick_ % 20 == 0) {
      std_msgs::msg::Float64MultiArray z;
      z.data = {0.0, 0.0, 0.0, 0.0};
      pub_wheel_->publish(z);
      z.data = {0.0, 0.0};
      pub_steer_->publish(z);
    }

    av_feature_msgs::msg::StuntStatus st;
    st.header = m.header;
    st.state = state_;
    st.t_state = t - t_state0_;
    st.t_mission = tm;
    st.height = h;
    st.vz = vz;
    st.roll_progress = roll_progress_;
    st.attitude_error = att_err;
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
    Vec3 com;
    Quat q;
  };

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
  double landed_z_, mission_timeout_;
  double lambda_;
  Vec6 w_flight_, w_brake_;
  std::unique_ptr<ThrustAllocator> alloc_;

  std::optional<double> feed_ratio_;
  double tank_mass_ = 0.0;
  double impulse_ = 0.0;
  double last_thrust_t_ = -1.0;
  std::optional<Prev> prev_;
  double roll_progress_ = 0.0;
  bool mission_started_ = false;
  double t_mission0_ = 0.0, t_state0_ = 0.0;
  std::string state_ = "WAIT";
  std::unique_ptr<FlipReference> flip_;
  unsigned tick_ = 0;

  rclcpp::Publisher<ros_gz_interfaces::msg::Float32Array>::SharedPtr pub_thr_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fan_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheel_, pub_steer_;
  rclcpp::Publisher<av_feature_msgs::msg::StuntStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_feed_, sub_mass_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Float32Array>::SharedPtr sub_thrust_;
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
