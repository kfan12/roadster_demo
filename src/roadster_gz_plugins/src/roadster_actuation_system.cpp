// roadster_actuation_system.cpp - the roadster's ONLY force-writing system.
//
// One Gazebo Fortress system owns the compressed-air tank, the 10 cold-gas
// nozzles and the suction fan (plan sections 5.2-5.4). Keeping every stunt
// force in one system means a single writer per step: no cross-system update
// order to depend on (Fortress has no system priorities) and no risk of one
// plugin overwriting another's wrench.
//
// Forces go through Link::AddWorldForce(force, offset-from-link-COM), which
// accumulates into ExternalWorldWrenchCmd with the torque for that point of
// application (the offset semantics were pinned down empirically, see
// Configure). tools/avkit/roadster_actuation_test.py checks in zero gravity
// that the resulting linear and angular accelerations match r x F about the
// COM, nozzle by nozzle.
//
// gz-transport interface (bridged to ROS by bridge_roadster.yaml):
//   sub /roadster/thruster_cmd   Float_V, one entry per nozzle in nozzles.yaml
//                                order (sorted by name), 0..1
//   sub /roadster/fan_cmd        Double, 0..1
//   pub /roadster/thruster_state Float_V, actual thrust per nozzle [N]
//   pub /roadster/tank/pressure  Double [Pa]
//   pub /roadster/tank/mass      Double [kg] gas remaining
//   pub /roadster/fan_force      Double [N]
//   pub /roadster/fan_gap        Double [m] (inf -> no surface under the car)
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/float_v.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <yaml-cpp/yaml.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "roadster_gz_plugins/fan_model.hpp"
#include "roadster_gz_plugins/nozzle_model.hpp"
#include "roadster_gz_plugins/tank_model.hpp"

namespace roadster_gz_plugins
{

namespace gzs = ignition::gazebo;
namespace gzm = ignition::math;

namespace
{

template<typename T>
T sdfGet(const std::shared_ptr<const sdf::Element> & sdf, const std::string & key, T def)
{
  return sdf->HasElement(key) ? sdf->Get<T>(key) : def;
}

std::shared_ptr<const sdf::Element> sdfChild(
  const std::shared_ptr<const sdf::Element> & sdf, const std::string & key)
{
  // FindElement on a const element: GetElement would add a default child
  return sdf->HasElement(key) ? sdf->FindElement(key) : nullptr;
}

Vec3 toVec3(const gzm::Vector3d & v) {return {v.X(), v.Y(), v.Z()};}

struct Nozzle
{
  std::string name;
  gzm::Vector3d pos_chassis;
  gzm::Vector3d dir_chassis;  // unit force direction on the car
  double f_max_mg = 0.0;
  std::unique_ptr<NozzleModel> model;
};

SurfaceSet loadSurfaces(const std::string & path)
{
  SurfaceSet s;
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot open surfaces file " + path);
  }
  const auto j = nlohmann::json::parse(f);
  for (const auto & e : j.at("surfaces")) {
    const std::string type = e.at("type");
    if (type == "plane") {
      PlaneSurface p;
      auto v = [](const nlohmann::json & a) {return Vec3{a[0], a[1], a[2]};};
      p.point = v(e.at("point"));
      p.normal = v(e.at("normal"));
      p.lo = v(e.at("box_min"));
      p.hi = v(e.at("box_max"));
      s.planes.push_back(p);
    } else if (type == "arc_y") {
      ArcSurface a;
      a.cx = e.at("center_xz")[0];
      a.cz = e.at("center_xz")[1];
      a.radius = e.at("radius");
      a.theta0 = e.at("theta0");
      a.theta1 = e.at("theta1");
      a.y_min = e.at("y_range")[0];
      a.y_max = e.at("y_range")[1];
      s.arcs.push_back(a);
    } else {
      throw std::runtime_error("unknown surface type '" + type + "' in " + path);
    }
  }
  return s;
}

}  // namespace

class RoadsterActuationSystem
  : public gzs::System,
  public gzs::ISystemConfigure,
  public gzs::ISystemPreUpdate
{
public:
  void Configure(
    const gzs::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gzs::EntityComponentManager & ecm, gzs::EventManager &) override
  {
    model_ = gzs::Model(entity);
    if (!model_.Valid(ecm)) {
      ignerr << "[roadster_actuation] must be attached to a model\n";
      return;
    }
    const std::string link_name = sdfGet<std::string>(sdf, "link_name", "base_link");
    link_ = gzs::Link(model_.LinkByName(ecm, link_name));
    if (!link_.Valid(ecm)) {
      ignerr << "[roadster_actuation] link '" << link_name << "' not found\n";
      return;
    }
    // Link::WorldPose() is empty unless the link carries a WorldPose component,
    // which the physics system only fills in once it exists
    if (!ecm.Component<gzs::components::WorldPose>(link_.Entity())) {
      ecm.CreateComponent(link_.Entity(), gzs::components::WorldPose());
    }
    frame_offset_ = sdfGet<gzm::Vector3d>(sdf, "frame_offset", gzm::Vector3d::Zero);

    // Link::AddWorldForce(force, position) treats `position` as an offset from
    // the link's COM (expressed in link axes), NOT a link-frame point: passing
    // link-frame points doubled the rear-main pitch lever in the zero-g test
    // (0.63 m instead of 0.30 m, 2026-10-16). Every application point is
    // therefore shifted by the link's inertial position once, here.
    if (auto in = ecm.Component<gzs::components::Inertial>(link_.Entity())) {
      com_link_ = in->Data().Pose().Pos();
    }

    // total model mass: every force parameter is a multiple of m g
    total_mass_ = 0.0;
    for (const auto & l : model_.Links(ecm)) {
      if (auto in = ecm.Component<gzs::components::Inertial>(l)) {
        total_mass_ += in->Data().MassMatrix().Mass();
      }
    }
    const double weight = total_mass_ * 9.81;

    // tank
    TankParams tp;
    if (auto t = sdfChild(sdf, "tank")) {
      tp.volume_m3 = sdfGet<double>(t, "volume_m3", tp.volume_m3);
      tp.p0_pa = sdfGet<double>(t, "p0_bar", tp.p0_pa / 1e5) * 1e5;
      tp.temperature_k = sdfGet<double>(t, "temperature_k", tp.temperature_k);
      tp.p_reg_pa = sdfGet<double>(t, "p_reg_bar", tp.p_reg_pa / 1e5) * 1e5;
      tp.p_min_pa = sdfGet<double>(t, "p_min_bar", tp.p_min_pa / 1e5) * 1e5;
      tp.regulated = sdfGet<std::string>(t, "feed_mode", "regulated") == "regulated";
    }
    tank_ = std::make_unique<TankModel>(tp);

    // nozzles
    NozzleParams np;
    if (auto n = sdfChild(sdf, "nozzle_defaults")) {
      np.tau_rise_s = sdfGet<double>(n, "tau_rise_s", np.tau_rise_s);
      np.tau_fall_s = sdfGet<double>(n, "tau_fall_s", np.tau_fall_s);
      np.min_on_time_s = sdfGet<double>(n, "min_on_time_s", np.min_on_time_s);
      np.exhaust_velocity = sdfGet<double>(n, "exhaust_velocity", np.exhaust_velocity);
    }
    const std::string nozzles_file = sdfGet<std::string>(sdf, "nozzles_file", "");
    try {
      const YAML::Node root = YAML::LoadFile(nozzles_file)["nozzles"];
      std::map<std::string, YAML::Node> sorted;  // same order as the xacro frames
      for (const auto & kv : root) {
        sorted[kv.first.as<std::string>()] = kv.second;
      }
      for (const auto & [name, n] : sorted) {
        Nozzle z;
        z.name = name;
        const auto xyz = n["xyz"].as<std::vector<double>>();
        const auto dir = n["force_dir"].as<std::vector<double>>();
        z.pos_chassis = {xyz.at(0), xyz.at(1), xyz.at(2)};
        z.dir_chassis = gzm::Vector3d(dir.at(0), dir.at(1), dir.at(2)).Normalized();
        z.f_max_mg = n["f_max_mg"].as<double>();
        NozzleParams p = np;
        p.f_max_n = z.f_max_mg * weight;
        z.model = std::make_unique<NozzleModel>(p);
        nozzles_.push_back(std::move(z));
      }
    } catch (const std::exception & e) {
      ignerr << "[roadster_actuation] nozzles_file '" << nozzles_file << "': " << e.what() << "\n";
      nozzles_.clear();
    }
    cmd_.assign(nozzles_.size(), 0.0);

    // fan
    FanParams fp;
    double fan_w0 = 0.0;
    if (auto f = sdfChild(sdf, "fan")) {
      fp.f_max_n = sdfGet<double>(f, "f_max_mg", 0.0) * weight;
      fp.tau_s = sdfGet<double>(f, "tau_s", fp.tau_s);
      fp.h_seal_m = sdfGet<double>(f, "h_seal_m", fp.h_seal_m);
      fp.h_max_m = sdfGet<double>(f, "h_max_m", fp.h_max_m);
      fan_w0 = sdfGet<double>(f, "initial_speed", 0.0);
      fan_cmd_ = sdfGet<double>(f, "initial_cmd", fan_w0);
      fan_point_chassis_ = sdfGet<gzm::Vector3d>(f, "point", gzm::Vector3d(0, 0, -0.075));
      fan_max_angle_ = IGN_DTOR(sdfGet<double>(f, "max_surface_angle_deg", 45.0));
    }
    fan_ = std::make_unique<FanModel>(fp, fan_w0);

    // surfaces for the fan gap: world JSON, or a default z = 0 ground plane
    const std::string surfaces_file = sdfGet<std::string>(sdf, "surfaces_file", "");
    if (!surfaces_file.empty()) {
      try {
        surfaces_ = loadSurfaces(surfaces_file);
      } catch (const std::exception & e) {
        ignerr << "[roadster_actuation] " << e.what() << "; using a z = 0 ground plane\n";
      }
    }
    if (surfaces_.planes.empty() && surfaces_.arcs.empty()) {
      const double big = 1e6;
      surfaces_.planes.push_back({{0, 0, 0}, {0, 0, 1}, {-big, -big, -big}, {big, big, big}});
    }

    state_period_ = 1.0 / sdfGet<double>(sdf, "state_rate_hz", 100.0);

    node_.Subscribe("/roadster/thruster_cmd", &RoadsterActuationSystem::onThrusterCmd, this);
    node_.Subscribe("/roadster/fan_cmd", &RoadsterActuationSystem::onFanCmd, this);
    pub_thrust_ = node_.Advertise<ignition::msgs::Float_V>("/roadster/thruster_state");
    pub_pressure_ = node_.Advertise<ignition::msgs::Double>("/roadster/tank/pressure");
    pub_mass_ = node_.Advertise<ignition::msgs::Double>("/roadster/tank/mass");
    pub_fan_ = node_.Advertise<ignition::msgs::Double>("/roadster/fan_force");
    pub_gap_ = node_.Advertise<ignition::msgs::Double>("/roadster/fan_gap");

    ignmsg << "[roadster_actuation] model mass " << total_mass_ << " kg, " << nozzles_.size() <<
      " nozzles, tank " << tank_->mass() << " kg at " << tank_->pressure() / 1e5 << " bar ("
           << (tp.regulated ? "regulated" : "blowdown") << "), fan f_max " << fp.f_max_n <<
      " N, " << surfaces_.planes.size() << " planes + " << surfaces_.arcs.size() << " arcs\n";
    configured_ = true;
  }

  void PreUpdate(const gzs::UpdateInfo & info, gzs::EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {
      return;
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (dt <= 0) {
      return;
    }
    const auto pose = link_.WorldPose(ecm);
    if (!pose) {
      return;
    }
    const gzm::Quaterniond & q = pose->Rot();

    std::vector<double> cmd;
    double fan_cmd;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      cmd = cmd_;
      fan_cmd = fan_cmd_;
    }

    // nozzles: thrust at the current feed pressure, then draw the gas
    const double p_feed = tank_->feedPressure();
    const double p_ref = tank_->referencePressure();
    double dm = 0.0;
    for (std::size_t i = 0; i < nozzles_.size(); ++i) {
      auto & z = nozzles_[i];
      const double f = z.model->step(cmd[i], p_feed, p_ref, t, dt);
      dm += z.model->massFlow() * dt;
      if (f > 0) {
        link_.AddWorldForce(
          ecm, q.RotateVector(z.dir_chassis * f), frame_offset_ + z.pos_chassis - com_link_);
      }
    }
    tank_->draw(dm);

    // fan: suction along body -z, sized by the analytic gap
    const gzm::Vector3d fan_point_link = frame_offset_ + fan_point_chassis_;
    const gzm::Vector3d fan_point_world = pose->Pos() + q.RotateVector(fan_point_link);
    const gzm::Vector3d body_z = q.RotateVector(gzm::Vector3d::UnitZ);
    const GapResult g = surfaces_.gap(toVec3(fan_point_world), toVec3(body_z), fan_max_angle_);
    const double ff = fan_->step(fan_cmd, g.h, dt);
    if (ff > 0) {
      link_.AddWorldForce(ecm, -body_z * ff, fan_point_link - com_link_);
    }

    if (t - last_state_t_ >= state_period_) {
      last_state_t_ = t;
      ignition::msgs::Float_V th;
      for (const auto & z : nozzles_) {
        th.add_data(static_cast<float>(z.model->thrust()));
      }
      pub_thrust_.Publish(th);
      publishDouble(pub_pressure_, tank_->pressure());
      publishDouble(pub_mass_, tank_->mass());
      publishDouble(pub_fan_, ff);
      publishDouble(pub_gap_, g.h);
    }
  }

private:
  void onThrusterCmd(const ignition::msgs::Float_V & msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (static_cast<std::size_t>(msg.data_size()) != cmd_.size()) {
      ignwarn << "[roadster_actuation] thruster_cmd has " << msg.data_size() <<
        " entries, expected " << cmd_.size() << "; ignored\n";
      return;
    }
    for (int i = 0; i < msg.data_size(); ++i) {
      cmd_[i] = msg.data(i);
    }
  }

  void onFanCmd(const ignition::msgs::Double & msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    fan_cmd_ = msg.data();
  }

  static void publishDouble(ignition::transport::Node::Publisher & p, double v)
  {
    ignition::msgs::Double m;
    m.set_data(v);
    p.Publish(m);
  }

  bool configured_ = false;
  gzs::Model model_{gzs::kNullEntity};
  gzs::Link link_{gzs::kNullEntity};
  gzm::Vector3d frame_offset_;
  gzm::Vector3d com_link_;  // link COM in the link frame
  double total_mass_ = 0.0;

  std::unique_ptr<TankModel> tank_;
  std::vector<Nozzle> nozzles_;
  std::unique_ptr<FanModel> fan_;
  gzm::Vector3d fan_point_chassis_{0, 0, -0.075};
  double fan_max_angle_ = IGN_DTOR(45.0);
  SurfaceSet surfaces_;

  std::mutex mutex_;
  std::vector<double> cmd_;
  double fan_cmd_ = 0.0;

  ignition::transport::Node node_;
  ignition::transport::Node::Publisher pub_thrust_, pub_pressure_, pub_mass_, pub_fan_, pub_gap_;
  double state_period_ = 0.01;
  double last_state_t_ = -1.0;
};

}  // namespace roadster_gz_plugins

IGNITION_ADD_PLUGIN(
  roadster_gz_plugins::RoadsterActuationSystem,
  ignition::gazebo::System,
  roadster_gz_plugins::RoadsterActuationSystem::ISystemConfigure,
  roadster_gz_plugins::RoadsterActuationSystem::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(
  roadster_gz_plugins::RoadsterActuationSystem,
  "roadster_gz_plugins::RoadsterActuationSystem")
