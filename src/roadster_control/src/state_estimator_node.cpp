// state_estimator_node.cpp - IMU-based motion estimate for the stunt roadster.
//
// Strapdown integration of /imu (gyro + accelerometer, 200 Hz) in an
// error-state EKF (ins_ekf.hpp), started at the world JSON's spawn pose, and
// corrected only by what a real car can sense about itself:
//   - wheels loaded (force/torque sensors): wheel speed along the body x axis
//     (encoder rate + gyro pitch rate) plus no side-slip / no hop (body y and
//     z velocity ~ 0), and each loaded wheel centre one radius from the
//     known track surface (maps/<world>.json, the
//     same analytic geometry the fan uses) - this pins height on the ramp
//   - standing still (wheels loaded, wheel speed ~ 0, gyro quiet): zero rate,
//     velocity, and gravity levelling of the attitude + accelerometer bias
//   - airborne: IMU only (about 2 s from release to touchdown)
// The IMU's orientation field is ignored (Gazebo fills it with ground truth).
//
// Publishes /odom_est (nav_msgs/Odometry, frame odom = world, child base_link,
// same convention as /odom_truth, twist in the child frame) at publish_rate
// (200 Hz); the filter itself integrates every 1 kHz IMU sample.
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

#include "roadster_control/ins_ekf.hpp"
#include "roadster_gz_plugins/surfaces_json.hpp"

namespace roadster_control
{

namespace
{
const std::array<const char *, 4> kWheelShort{"fl", "fr", "rl", "rr"};
const std::array<const char *, 4> kWheelJoint{
  "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint",
  "rear_right_wheel_joint"};
const std::array<const char *, 4> kSuspJoint{
  "front_left_suspension_joint", "front_right_suspension_joint", "rear_left_suspension_joint",
  "rear_right_suspension_joint"};
}  // namespace

class StateEstimatorNode : public rclcpp::Node
{
public:
  StateEstimatorNode()
  : Node("state_estimator")
  {
    const std::string world_json = declare_parameter<std::string>("world_json", "");
    chassis_z_ = declare_parameter<double>("chassis_offset_z", 0.33);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.33);
    const double wb = declare_parameter<double>("wheelbase", 2.7);
    const double tw = declare_parameter<double>("track_width", 1.6);
    // wheel centres in the chassis (= IMU) frame, FL FR RL RR (roadster.urdf.xacro)
    wheel_pos_ = {InsEkf::Vec3(wb / 2, tw / 2, 0), InsEkf::Vec3(wb / 2, -tw / 2, 0),
      InsEkf::Vec3(-wb / 2, tw / 2, 0), InsEkf::Vec3(-wb / 2, -tw / 2, 0)};
    contact_n_ = declare_parameter<double>("contact_normal", 600.0);
    update_rate_ = declare_parameter<double>("update_rate", 50.0);
    sig_wheel_ = declare_parameter<double>("sigma_wheel_speed", 0.15);
    sig_nhc_ = declare_parameter<double>("sigma_no_slip", 0.08);
    sig_surface_ = declare_parameter<double>("sigma_surface", 0.02);
    sig_zupt_ = declare_parameter<double>("sigma_zupt", 0.02);
    sig_grav_ = declare_parameter<double>("sigma_gravity", 0.3);
    sig_zaru_ = declare_parameter<double>("sigma_zero_rate", 0.02);
    use_wheel_ = declare_parameter<bool>("use_wheel_speed", true);
    use_nhc_ = declare_parameter<bool>("use_no_slip", true);
    use_surface_ = declare_parameter<bool>("use_surface", true);
    static_time_ = declare_parameter<double>("static_time", 0.3);
    max_reject_time_ = declare_parameter<double>("max_reject_time", 0.2);
    publish_rate_ = declare_parameter<double>("publish_rate", 200.0);
    InsNoise n;
    n.accel = declare_parameter<double>("noise_accel", n.accel);
    n.gyro = declare_parameter<double>("noise_gyro", n.gyro);
    ekf_ = InsEkf(n);
    ekf_.setGating(declare_parameter<bool>("innovation_gating", true));

    std::ifstream f(world_json);
    if (!f) {
      throw std::runtime_error("state_estimator needs world_json, cannot open '" + world_json + "'");
    }
    const auto j = nlohmann::json::parse(f);
    const auto & s = j.at("spawn");
    spawn_p_ = InsEkf::Vec3(s.at("x"), s.at("y"), s.at("z"));
    spawn_q_ = InsEkf::Quat(Eigen::AngleAxisd(s.value("yaw", 0.0), InsEkf::Vec3::UnitZ()) *
      Eigen::AngleAxisd(s.value("pitch", 0.0), InsEkf::Vec3::UnitY()) *
      Eigen::AngleAxisd(s.value("roll", 0.0), InsEkf::Vec3::UnitX()));
    surfaces_ = roadster_gz_plugins::loadSurfacesJson(world_json);

    pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom_est", 50);
    for (std::size_t i = 0; i < 4; ++i) {
      sub_wrench_[i] = create_subscription<geometry_msgs::msg::Wrench>(
        std::string("/roadster/wheel_wrench/") + kWheelShort[i], 50,
        [this, i](geometry_msgs::msg::Wrench::SharedPtr m) {
          wheel_force_[i] = InsEkf::Vec3(m->force.x, m->force.y, m->force.z);
        });
    }
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50, [this](sensor_msgs::msg::JointState::SharedPtr m) {
        for (std::size_t k = 0; k < m->name.size(); ++k) {
          for (std::size_t i = 0; i < 4; ++i) {
            if (m->name[k] == kWheelJoint[i]) {
              if (k < m->position.size()) {wheel_angle_[i] = m->position[k];}
              if (k < m->velocity.size()) {wheel_rate_[i] = m->velocity[k];}
            } else if (m->name[k] == kSuspJoint[i] && k < m->position.size()) {
              susp_[i] = m->position[k];  // +: wheel up relative to the chassis
            }
          }
        }
        have_js_ = true;
      });
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 1000, [this](sensor_msgs::msg::Imu::SharedPtr m) {onImu(*m);});
    RCLCPP_INFO(get_logger(), "IMU state estimator: start (%.2f %.2f %.2f), %zu planes + %zu arcs",
      spawn_p_.x(), spawn_p_.y(), spawn_p_.z(), surfaces_.planes.size(), surfaces_.arcs.size());
  }

private:
  // same sign convention as stunt_mission_node / roadster_hold_test.py
  double wheelNormal(std::size_t i) const
  {
    const double a = wheel_angle_[i];
    return -std::sin(a) * wheel_force_[i].x() + std::cos(a) * wheel_force_[i].z();
  }

  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const InsEkf::Vec3 f(m.linear_acceleration.x, m.linear_acceleration.y,
      m.linear_acceleration.z);
    const InsEkf::Vec3 w(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
    const InsEkf::Vec3 lever(0, 0, -chassis_z_);  // IMU (chassis origin) -> base_link

    if (!t_prev_) {
      // the IMU sits at the chassis origin, chassis_z above base_link
      ekf_.reset(spawn_p_ - spawn_q_ * lever, spawn_q_, InsEkf::Vec3::Zero(),
        0.02, 0.02, 0.01, 0.02, 0.01);  // accel-bias sigma at the IMU model scale
      t_prev_ = t;
      return;
    }
    const double dt = t - *t_prev_;
    t_prev_ = t;
    if (dt <= 0.0 || dt > 0.1) {
      return;
    }
    ekf_.propagate(f, w, dt);

    if (have_js_ && t - t_update_ >= 1.0 / update_rate_) {
      t_update_ = t;
      std::array<bool, 4> on{};
      int loaded = 0;
      double rate = 0.0;
      for (std::size_t i = 0; i < 4; ++i) {
        on[i] = wheelNormal(i) > contact_n_;
        if (on[i]) {
          ++loaded;
          rate += wheel_rate_[i];
        }
      }
      if (loaded >= 3) {
        // Ground speed from the wheels: the encoders measure spin RELATIVE to
        // the chassis, which itself pitches at v / (R - r) round the arc, so
        // the absolute wheel rate is joint rate + body pitch rate (gyro y).
        // Without this the wheels over-read by r / (R - r) ~ 4% on the 8 m
        // loop, and that biased update bent the attitude (yaw ran away at up
        // to 80 deg/s in the first shadow run). The IMU sits midway between
        // the axles, so the mean wheel-centre speed is the IMU point's speed.
        const double w_pitch = w.y() - ekf_.bg().y();
        const double v_wheel = (rate / loaded + w_pitch) * wheel_radius_;
        if (use_wheel_ || use_nhc_) {
          // a switched-off axis gets a huge sigma (no information)
          const double sx = use_wheel_ ? sig_wheel_ : 1e3, syz = use_nhc_ ? sig_nhc_ : 1e3;
          // The gate rejects skidding / spinning wheels. But if the wheels are
          // rejected for max_reject_time while they are clearly loaded, the
          // ESTIMATE is what is wrong (an impact the 200 Hz IMU under-sampled,
          // a drift after a skid), and a pure gate would lock it out for good:
          // the estimate sank 3.7 m through the road after touchdown and crept
          // 0.6 m/s along a stopped overhang. Then force the update through.
          const bool force = t - velocity_ok_since_ > max_reject_time_;
          if (ekf_.updateBodyVelocity(InsEkf::Vec3(v_wheel, 0, 0),
            InsEkf::Vec3(sx, syz, syz), force))
          {
            velocity_ok_since_ = t;
          }
        }
        // Each loaded wheel's CENTRE is one wheel radius from the known track
        // surface, along its normal - true on the flat, the arc and the
        // overhang alike. (Constraining base_link, the mid-axle point, was
        // wrong on the arc: it sits on the chord, ~11.5 cm off the surface.)
        if (use_surface_) {
          for (std::size_t i = 0; i < 4; ++i) {
            if (!on[i]) {
              continue;
            }
            const auto R = ekf_.q().toRotationMatrix();
            // wheel centre moves with the suspension travel (up to ~8 cm on the
            // climb; ignoring it would bias height and pitch)
            const InsEkf::Vec3 wc = wheel_pos_[i] + InsEkf::Vec3(0, 0, susp_[i]);
            const InsEkf::Vec3 pc = ekf_.p() + R * wc;
            const InsEkf::Vec3 bz = R.col(2);
            const auto g = surfaces_.gap({pc.x(), pc.y(), pc.z()}, {bz.x(), bz.y(), bz.z()}, 0.6);
            if (std::isfinite(g.h) && std::abs(g.h - wheel_radius_) < 1.0) {
              ekf_.updateSurface(g.h - wheel_radius_,
                InsEkf::Vec3(g.normal.x, g.normal.y, g.normal.z), wc, sig_surface_);
            }
          }
        }
        // standstill only once wheels, gyro and accelerometer have ALL been
        // quiet for static_time: locked wheels on a sliding car stop the
        // encoders but not the accelerometer
        const bool quiet = std::abs(v_wheel) < 0.02 && w.norm() < 0.05 &&
          std::abs(f.norm() - InsEkf::kG) < 0.5;
        if (!quiet) {
          quiet_since_ = t;
        }
        if (quiet && t - quiet_since_ >= static_time_) {
          ekf_.updateZeroVelocity(sig_zupt_);
          ekf_.updateGravity(f, sig_grav_);
          ekf_.updateZeroRate(w, sig_zaru_);
        }
      } else {
        quiet_since_ = t;
        velocity_ok_since_ = t;  // the rejection clock only runs while loaded
      }
    }

    // integrate every IMU sample (1 kHz), publish at publish_rate: the mission
    // runs its control and allocation on every pose it receives
    // on the publish_rate grid (sim time multiples of 1/rate, like the other
    // 200 Hz sim topics) so estimate and truth samples line up
    const long slot = std::lround(std::floor(t * publish_rate_ + 1e-6));
    if (slot == last_slot_) {
      return;
    }
    last_slot_ = slot;
    const auto R = ekf_.q().toRotationMatrix();
    const InsEkf::Vec3 pb = ekf_.p() + R * lever;
    const InsEkf::Vec3 wb = w - ekf_.bg();
    const InsEkf::Vec3 vb = R.transpose() * ekf_.v() + wb.cross(lever);  // base_link, body
    nav_msgs::msg::Odometry o;
    o.header.stamp = m.header.stamp;
    o.header.frame_id = "odom";
    o.child_frame_id = "base_link";
    o.pose.pose.position.x = pb.x();
    o.pose.pose.position.y = pb.y();
    o.pose.pose.position.z = pb.z();
    o.pose.pose.orientation.x = ekf_.q().x();
    o.pose.pose.orientation.y = ekf_.q().y();
    o.pose.pose.orientation.z = ekf_.q().z();
    o.pose.pose.orientation.w = ekf_.q().w();
    const auto & P = ekf_.P();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        o.pose.covariance[r * 6 + c] = P(r, c);
        o.pose.covariance[(r + 3) * 6 + c + 3] = P(6 + r, 6 + c);
        o.twist.covariance[r * 6 + c] = P(3 + r, 3 + c);
      }
    }
    o.twist.twist.linear.x = vb.x();
    o.twist.twist.linear.y = vb.y();
    o.twist.twist.linear.z = vb.z();
    o.twist.twist.angular.x = wb.x();
    o.twist.twist.angular.y = wb.y();
    o.twist.twist.angular.z = wb.z();
    pub_->publish(o);
    if (ekf_.rejected() != last_rejected_ && t - last_reject_log_ > 1.0) {
      RCLCPP_INFO(get_logger(), "[%.2f] innovation gate rejected %lu measurements so far",
        t, ekf_.rejected());
      last_rejected_ = ekf_.rejected();
      last_reject_log_ = t;
    }
  }

  InsEkf ekf_;
  InsEkf::Vec3 spawn_p_;
  InsEkf::Quat spawn_q_;
  roadster_gz_plugins::SurfaceSet surfaces_;
  double chassis_z_, wheel_radius_, contact_n_, update_rate_;
  double sig_wheel_, sig_nhc_, sig_surface_, sig_zupt_, sig_grav_, sig_zaru_;
  bool use_wheel_, use_nhc_, use_surface_;
  double static_time_, quiet_since_ = 0.0, max_reject_time_, velocity_ok_since_ = 0.0;
  double publish_rate_;
  long last_slot_ = -1;
  unsigned long last_rejected_ = 0;
  double last_reject_log_ = -10.0;
  std::array<InsEkf::Vec3, 4> wheel_pos_;
  std::array<double, 4> susp_{0, 0, 0, 0};
  std::optional<double> t_prev_;
  double t_update_ = -1.0;
  bool have_js_ = false;
  std::array<InsEkf::Vec3, 4> wheel_force_{};
  std::array<double, 4> wheel_angle_{0, 0, 0, 0}, wheel_rate_{0, 0, 0, 0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr, 4> sub_wrench_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
};

}  // namespace roadster_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roadster_control::StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
