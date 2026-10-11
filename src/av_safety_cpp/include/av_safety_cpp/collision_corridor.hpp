#pragma once
// Predicted ego paths + swept vehicle footprint vs. obstacle clusters

#include "av_common/geometry.hpp"
#include "depth_obstacle_extractor.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

namespace av::safety
{
    struct FootprintParams
    {
        double vehicle_length_m = 4.5;      // sedan.urdf.xacro ch_l
        double vehicle_width_m = 1.8;       // sedan.urdf.xacro ch_w
        double lateral_margin_m = 0.3;      // extra clearance beyond the vehicle footprint
        double longitudinal_margin_m = 0.5; // extra clearance beyond the vehicle footprint
    };

    // Independent kinematic prediction from current speed + yaw rate.
    // This is what keeps AEB alive when mpc_predicted_path is absent,
    // stale, or wrong - it never depends on the MPC, local_planner,
    // or lane_node being healthy. Output is in base_link (ego at the origin, heading +x).

    // using Polyline = std::vector<Eigen::Vector2d>;
    inline av::geom::Polyline kinematic_prediction(
        double v, double yaw_rate, double t_horizon, double dt)
    {
        av::geom::Polyline out;
        double x = 0, y = 0, psi = 0;
        out.push_back({x, y});
        for (double t = dt; t <= t_horizon; t += dt)
        {
            x += v * std::cos(psi) * dt;
            y += v * std::sin(psi) * dt;
            psi += yaw_rate * dt;
            out.push_back({x, y});
        }
        return out;
    }

    struct CorridorResult
    {
        bool obstacle_in_corridor = false;
        int nearest_obstacle_id = -1;    // index into clusters[] of the nearest intersecting cluster
        double nearest_distance_m = 1e9; // distance from the FRONT bumper to the nearest intersecting cluster's leading edge
        Eigen::Vector2d nearest_point{0, 0};
    };

    // path: predicted ego centerline in base_link (ego at s=0). clusters: from
    // extract_clusters, also base_link. The footprint rides the path like a
    // rectangle extruded along it: a cluster counts as "in corridor" if its
    // lateral offset from its OWN foot point on the path is within half the
    // vehicle width + margin.

    inline CorridorResult check_corridor(
        const av::geom::Polyline &path,
        const std::vector<ObstacleCluster> &clusters,
        const FootprintParams &fp)
    {
        CorridorResult result;
        if (path.size() < 2)
            return result; // no path, no corridor

        double half_w = fp.vehicle_width_m / 2.0 + fp.lateral_margin_m;
        double half_l = fp.vehicle_length_m / 2.0 + fp.longitudinal_margin_m;

        for (auto const &c : clusters)
        {
            Eigen::Vector2d q(c.centroid.x(), c.centroid.y());
            auto pr = av::geom::project_point(path, q);
            if (pr.dist > half_w)
                continue;             // laterally outside the swept corridor
            double dist_ahead = pr.s; // arc length from ego to the cluster's foot point
            if (dist_ahead < -half_l)
                continue; // longitudinally behind the corridor
            result.obstacle_in_corridor = true;
            double leading_edge = std::max(0.0, dist_ahead - half_l); // front bumper to cluster's leading edge
            if (leading_edge < result.nearest_distance_m)
            {
                result.nearest_distance_m = leading_edge;
                result.nearest_point = q;
                result.nearest_obstacle_id = &c - &clusters[0];
            }
        }

        return result;
    }

} // namespace av::safety
