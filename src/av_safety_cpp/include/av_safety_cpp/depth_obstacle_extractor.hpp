#pragma once

// Depth image -> clustered obstacle points in base_link. Deliberately
// independent of av_perception_cpp (AEB must not share fate with the lane
// pipeline): this header duplicates the small zero-copy depth-access pattern
// instead of depending on av_perception_cpp/img_access.hpp.

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <map>

#include "av_common/projection.hpp"

namespace av::safety
{
    struct ObstacleFilterParams
    {
        double min_depth_m = 0.5;
        double max_depth_m = 40.0;
        double max_lateral_m = 5.0;
        double min_height_m = 0.15;
        double max_height_m = 3.0;
        double ground_removal_m = 0.1;
        double cluster_voxel_m = 0.5;
        int min_cluster_points = 5;
    };

    struct ObstacleCluster
    {
        Eigen::Vector3d centroid{0, 0, 0}; // base_link frame; x() holds the NEAREST point's depth, not the mean
        int n_points = 0;
    };

    // same zero-copy aliasing as av_perception_cpp/img_access.hpp::as_depth,
    // duplicated on purpose
    inline cv::Mat depth_as_mat(const sensor_msgs::msg::Image::ConstSharedPtr &m)
    {
        return cv::Mat(m->height, m->width, CV_32FC1,
                       const_cast<uint8_t *>(m->data.data()), m->step);
    }

    inline std::vector<ObstacleCluster> extract_clusters(
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_image,
        const av::proj::Pinhole &pin,
        const Eigen::Isometry3d &T_cam_base,
        const ObstacleFilterParams &fp,
        int stride = 2)
    {
        cv::Mat depth = depth_as_mat(depth_image);
        std::map<std::pair<int, int>, std::vector<Eigen::Vector3d>> voxels;

        for (int v = 0; v < depth.rows; v += stride)
        {
            for (int u = 0; u < depth.cols; u += stride)
            {
                float d = depth.at<float>(v, u);
                if (!std::isfinite(d) || d < fp.min_depth_m || d > fp.max_depth_m)
                    continue;
                Eigen::Vector3d p_cam = av::proj::pixel_depth_to_camera(pin, u, v, d);
                Eigen::Vector3d p_base = T_cam_base * p_cam;
                if (std::abs(p_base.y()) > fp.max_lateral_m)
                    continue;

                // base_link sits at ground level (ground_cal.hpp's 2026-08-28
                // fix), so p_base.z() IS height above ground directly.
                double min_h = std::max(fp.ground_removal_m, fp.min_height_m);
                if (p_base.z() < min_h || p_base.z() > fp.max_height_m)
                    continue;

                int vx = static_cast<int>(std::floor(p_base.x() / fp.cluster_voxel_m));
                int vy = static_cast<int>(std::floor(p_base.y() / fp.cluster_voxel_m));
                voxels[{vx, vy}].push_back(p_base);
            }
        }

        std::vector<ObstacleCluster> out;
        for (const auto &[_, pts] : voxels)
        {
            if (static_cast<int>(pts.size()) < fp.min_cluster_points)
                continue;
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            double nearest_x = 1e9;
            for (const auto &p : pts)
            {
                sum += p;
                nearest_x = std::min(nearest_x, p.x());
            }
            ObstacleCluster cluster;
            cluster.centroid = sum / static_cast<double>(pts.size());
            cluster.centroid.x() = nearest_x;
            cluster.n_points = static_cast<int>(pts.size());
            out.push_back(cluster);
        }
        return out;
    }

} // namespace av::safety