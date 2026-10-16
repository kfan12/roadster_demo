// ins_replay.cpp - replay a recorded stunt run (record_stunt_run.py JSONL)
// through the IMU estimator's filter offline, and report its error against
// the recorded ground truth. Mirrors state_estimator_node's update logic, so
// tuning and model changes can be tried in seconds instead of a sim run.
//
//   ins_replay <run.jsonl> <maps/world.json> [key=value ...]
//     keys: sigma_wheel sigma_no_slip sigma_surface noise_accel noise_gyro
//           gating (0/1) max_reject_time use_wheel use_nhc use_surface
//           t0 t1 (report window) print_every
//
// Wheel contact uses |F| of the wheel F/T reading (the run log has no wheel
// joint angles), which is the same test the node makes in practice.
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "roadster_control/attitude.hpp"
#include "roadster_control/ins_ekf.hpp"
#include "roadster_gz_plugins/surfaces_json.hpp"

using roadster_control::InsEkf;
using roadster_control::InsNoise;
using Vec3 = InsEkf::Vec3;
using Quat = InsEkf::Quat;

struct Truth
{
  double t;
  Vec3 p;
  Quat q;
};

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: ins_replay run.jsonl world.json [key=value ...]\n");
    return 2;
  }
  std::map<std::string, double> cfg{
    {"sigma_wheel", 0.15}, {"sigma_no_slip", 0.08}, {"sigma_surface", 0.02},
    {"noise_accel", 0.08}, {"noise_gyro", 0.009}, {"gating", 1}, {"max_reject_time", 0.2},
    {"use_wheel", 1}, {"use_nhc", 1}, {"use_surface", 1}, {"t0", 0}, {"t1", 1e9},
    {"print_every", 0.5}, {"contact", 600}, {"rate", 50}};
  for (int i = 3; i < argc; ++i) {
    std::string a(argv[i]);
    const auto e = a.find('=');
    cfg[a.substr(0, e)] = std::stod(a.substr(e + 1));
  }

  struct Rec
  {
    double t;
    int kind;  // 0 imu, 1 wheel rate, 2 wheel force, 3 truth
    std::vector<double> v;
    int w = -1;
    Quat q;
  };
  std::vector<Rec> recs;
  std::ifstream in(argv[1]);
  std::string line;
  const std::map<std::string, int> wi{{"fl", 0}, {"fr", 1}, {"rl", 2}, {"rr", 3}};
  while (std::getline(in, line)) {
    const auto j = nlohmann::json::parse(line);
    const std::string tp = j.at("topic");
    Rec r;
    r.t = j.at("t");
    if (tp == "imu") {
      r.kind = 0;
      for (double x : j.at("f")) {r.v.push_back(x);}
      for (double x : j.at("w")) {r.v.push_back(x);}
    } else if (tp == "wheel_rate") {
      r.kind = 1;
      for (double x : j.at("w")) {r.v.push_back(x);}
    } else if (tp == "wheel") {
      r.kind = 2;
      r.w = wi.at(j.at("w").get<std::string>());
      for (double x : j.at("f")) {r.v.push_back(x);}
    } else if (tp == "odom") {
      r.kind = 3;
      for (double x : j.at("p")) {r.v.push_back(x);}
      const auto & qq = j.at("q");
      r.q = Quat(qq[3], qq[0], qq[1], qq[2]);
    } else {
      continue;
    }
    recs.push_back(r);
  }
  // the recorder stamps wheel data with the last status time: keep file order
  // for those, but merge by time (stable) so IMU / truth interleave correctly
  std::stable_sort(recs.begin(), recs.end(), [](const Rec & a, const Rec & b) {return a.t < b.t;});

  const auto surfaces = roadster_gz_plugins::loadSurfacesJson(argv[2]);
  InsNoise n;
  n.accel = cfg["noise_accel"];
  n.gyro = cfg["noise_gyro"];
  InsEkf f(n);
  f.setGating(cfg["gating"] > 0.5);
  const double r = 0.33;
  const Vec3 lever(0, 0, -0.33);
  const Vec3 wp[4] = {{1.35, 0.8, 0}, {1.35, -0.8, 0}, {-1.35, 0.8, 0}, {-1.35, -0.8, 0}};
  double rates[4] = {0, 0, 0, 0};
  double fmag[4] = {0, 0, 0, 0};
  bool init = false;
  double t_prev = 0, t_upd = -1, ok_since = 0, quiet_since = 0, last_print = -1;
  Truth truth{0, Vec3::Zero(), Quat::Identity()};
  bool have_truth = false;
  double max_att = 0, max_pos = 0, max_h = 0;

  for (const auto & rc : recs) {
    if (rc.kind == 1) {
      for (int k = 0; k < 4; ++k) {rates[k] = rc.v[k];}
      continue;
    }
    if (rc.kind == 2) {
      fmag[rc.w] = std::sqrt(rc.v[0] * rc.v[0] + rc.v[1] * rc.v[1] + rc.v[2] * rc.v[2]);
      continue;
    }
    if (rc.kind == 3) {
      truth = {rc.t, Vec3(rc.v[0], rc.v[1], rc.v[2]), rc.q};
      have_truth = true;
      if (!init) {
        f.reset(truth.p - truth.q * lever, truth.q, Vec3::Zero(), 0.02, 0.02, 0.01, 0.02, 0.01);
        init = true;
        t_prev = rc.t;
      }
      continue;
    }
    if (!init) {continue;}
    const Vec3 fs(rc.v[0], rc.v[1], rc.v[2]), w(rc.v[3], rc.v[4], rc.v[5]);
    const double t = rc.t, dt = t - t_prev;
    t_prev = t;
    if (dt <= 0 || dt > 0.1) {continue;}
    f.propagate(fs, w, dt);
    if (t - t_upd >= 1.0 / cfg["rate"]) {
      t_upd = t;
      int loaded = 0;
      double rate = 0;
      bool on[4];
      for (int k = 0; k < 4; ++k) {
        on[k] = fmag[k] > cfg["contact"];
        if (on[k]) {++loaded; rate += rates[k];}
      }
      if (loaded >= 3) {
        const double vw = (rate / loaded + w.y() - f.bg().y()) * r;
        const double sx = cfg["use_wheel"] > 0.5 ? cfg["sigma_wheel"] : 1e3;
        const double syz = cfg["use_nhc"] > 0.5 ? cfg["sigma_no_slip"] : 1e3;
        if (f.updateBodyVelocity(Vec3(vw, 0, 0), Vec3(sx, syz, syz),
          t - ok_since > cfg["max_reject_time"]))
        {
          ok_since = t;
        }
        if (cfg["use_surface"] > 0.5) {
          for (int k = 0; k < 4; ++k) {
            if (!on[k]) {continue;}
            const auto Rm = f.q().toRotationMatrix();
            const Vec3 pc = f.p() + Rm * wp[k];
            const Vec3 bz = Rm.col(2);
            const auto g = surfaces.gap({pc.x(), pc.y(), pc.z()}, {bz.x(), bz.y(), bz.z()}, 0.6);
            if (std::isfinite(g.h) && std::abs(g.h - r) < 1.0) {
              f.updateSurface(g.h - r, Vec3(g.normal.x, g.normal.y, g.normal.z), wp[k],
                cfg["sigma_surface"]);
            }
          }
        }
        const bool quiet = std::abs(vw) < 0.02 && w.norm() < 0.05 &&
          std::abs(fs.norm() - InsEkf::kG) < 0.5;
        if (!quiet) {quiet_since = t;}
        if (quiet && t - quiet_since >= 0.3) {
          f.updateZeroVelocity(0.02);
          f.updateGravity(fs, 0.3);
          f.updateZeroRate(w, 0.02);
        }
      } else {
        quiet_since = t;
        ok_since = t;
      }
    }
    if (have_truth && t >= cfg["t0"] && t <= cfg["t1"] && std::abs(t - truth.t) < 0.003) {
      const Vec3 pb = f.p() + f.q() * lever;
      const Vec3 e = roadster_control::attitudeError(truth.q, f.q());
      max_att = std::max(max_att, e.norm());
      max_pos = std::max(max_pos, (pb - truth.p).norm());
      max_h = std::max(max_h, std::abs(pb.z() - truth.p.z()));
      if (t - last_print >= cfg["print_every"]) {
        last_print = t;
        std::printf("t=%6.2f err pos %6.3f m (dx %+6.3f dz %+6.3f)  att body (%+5.2f %+5.2f %+5.2f) deg"
          "  bg_y %+.4f ba (%+.3f %+.3f %+.3f)\n", t, (pb - truth.p).norm(), pb.x() - truth.p.x(),
          pb.z() - truth.p.z(), e.x() * 57.3, e.y() * 57.3, e.z() * 57.3, f.bg().y(),
          f.ba().x(), f.ba().y(), f.ba().z());
      }
    }
  }
  std::printf("MAX pos %.3f m  height %.3f m  attitude %.2f deg  rejected %lu\n", max_pos, max_h,
    max_att * 57.3, f.rejected());
  return 0;
}
