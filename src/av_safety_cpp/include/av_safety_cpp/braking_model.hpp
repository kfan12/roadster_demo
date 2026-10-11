#pragma once
// Time-to-collision + safe braking distance.
#include <algorithm>

namespace av::safety
{
    inline double time_to_collision(double distance_m, double v_rel_mps, double eps = 0.05)
    {
        return distance_m / std::max(v_rel_mps, eps);
    }

    // d_safe = v*t_response + v^2/(2*a_decel) + d_offset.
    // Stationary-obstacle form. Tracking a moving lead
    // object's own deceleration needs cross-frame cluster association.
    inline double safe_braking_distance(double v_mps, double t_response_s,
                                        double a_decel_mps2, double d_offset_m)
    {
        double a = std::max(a_decel_mps2, 0.1);
        return v_mps * t_response_s + (v_mps * v_mps) / (2.0 * a) + d_offset_m;
    }
} // namespace av::safety