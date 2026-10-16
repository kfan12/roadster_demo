// surfaces_json.hpp - load the analytic stunt surfaces from maps/<world>.json.
//
// Shared by the actuation system (fan gap) and roadster_control's state
// estimator (wheels-on-surface update), so both read the generator's geometry
// the same way. Header-only; needs nlohmann/json.
#pragma once

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>
#include <string>

#include "roadster_gz_plugins/fan_model.hpp"

namespace roadster_gz_plugins
{

inline SurfaceSet loadSurfacesJson(const std::string & path)
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

}  // namespace roadster_gz_plugins
