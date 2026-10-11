#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "av_common/projection.hpp"
#include "av_safety_cpp/braking_model.hpp"
#include "av_safety_cpp/collision_corridor.hpp"
#include "av_safety_cpp/depth_obstacle_extractor.hpp"
#include <av_feature_msgs/msg/aeb_status.hpp>

using av_feature_msgs::msg::AebStatus;

enum class AebState : uint8_t
{
    INACTIVE = AebStatus::INACTIVE,
    MONITORING = AebStatus::MONITORING,
    WARNING = AebStatus::WARNING,
    BRAKING = AebStatus::BRAKING,
    STOP_HOLD = AebStatus::STOP_HOLD,
    FAULT = AebStatus::FAULT,
};

class AebNode : public rclcpp::Node
{
public:
    AebNode() : rclcpp::Node("aeb_node")
    {
        // camera extrinsic/intrinsic - mirrors sedan.urdf.xacro, the same
        // duplication discipline every av_perception_cpp node follows
        double cam_x = declare_parameter("cam_x", 2.2);
        double cam_z = declare_parameter("cam_z", 1.4);
        double cam_pitch = declare_parameter("cam_pitch", 0.06);
        int img_w = declare_parameter("image_width", 424);
        int img_h = declare_parameter("image_height", 300);
        double hfov = declare_parameter("horizontal_fov", 1.9);

        T_base_cam_ = av::proj::make_T_base_cam(cam_x, 0.0, cam_z, cam_pitch);
        double fx = (img_w / 2.0) / std::tan(hfov / 2.0); // derived, never read from camera_info (ground rule)
        pin_ = {fx, fx, img_w / 2.0, img_h / 2.0};

        fp_.min_depth_m = declare_parameter("min_depth_m", 0.5);
        fp_.max_depth_m = declare_parameter("max_depth_m", 40.0);
        fp_.max_lateral_m = declare_parameter("max_lateral_m", 5.0);
        fp_.min_height_m = declare_parameter("min_obstacle_height_m", 0.15);
        fp_.max_height_m = declare_parameter("max_obstacle_height_m", 3.0);
        fp_.ground_removal_m = declare_parameter("ground_removal_threshold_m", 0.1);
        fp_.cluster_voxel_m = declare_parameter("cluster_voxel_m", 0.5);
        fp_.min_cluster_points = declare_parameter("minimum_cluster_points", 5);

        pred_t_ = declare_parameter("prediction_time_sec", 3.0);
        pred_dt_ = declare_parameter("prediction_dt_sec", 0.1);
        mpc_path_timeout_ = declare_parameter("mpc_path_timeout_sec", 0.3);

        foot_.vehicle_length_m = declare_parameter("vehicle_length_m", 4.5);
        foot_.vehicle_width_m = declare_parameter("vehicle_width_m", 1.8);
        foot_.lateral_margin_m = declare_parameter("lateral_margin_m", 0.3);
        foot_.longitudinal_margin_m = declare_parameter("longitudinal_margin_m", 0.5);

        t_response_ = declare_parameter("response_time_sec", 0.3);
        a_decel_ = declare_parameter("ego_emergency_decel_mps2", 6.0);
        d_offset_ = declare_parameter("distance_offset_m", 1.0);

        warning_ttc_ = declare_parameter("warning_ttc_sec", 2.5);
        brake_ttc_ = declare_parameter("brake_ttc_sec", 1.5);
        collision_hold_ = declare_parameter("collision_hold_sec", 1.0);
        clear_hysteresis_ = declare_parameter("clear_hysteresis_sec", 0.5);
        persistence_ticks_ = declare_parameter("persistence_ticks", 3);
        stop_hold_speed_ = declare_parameter("stop_hold_speed_mps", 0.1);

        rate_hz_ = declare_parameter("aeb_rate_hz", 20.0);
        odom_stale_ = declare_parameter("odom_stale_sec", 0.3);
        depth_stale_ = declare_parameter("depth_stale_sec", 0.3);

        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/front/depth/image_raw", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr m)
            { depth_ = m; depth_t_ = now(); });

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_ekf", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m)
            { odom_ = m; odom_t_ = now(); });

        sub_mpc_path_ = create_subscription<nav_msgs::msg::Path>(
            "/mpc_predicted_path", 10,
            [this](nav_msgs::msg::Path::ConstSharedPtr m)
            { mpc_path_ = m; mpc_path_t_ = now(); });

        // Remap-friendly handle on the nominal controller's last command, so
        // BRAKING can preserve the current yaw rate instead of zeroing it
        // mid-curve. Today this is remapped to /cmd_vel
        // (MPC still publishes there directly); Later plan moves the remap target
        // to /control/mpc_cmd when the arbiter takes over /cmd_vel - this
        // node's code does not change between the two.
        sub_nominal_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "nominal_cmd", 10,
            [this](geometry_msgs::msg::Twist::ConstSharedPtr m)
            { last_nominal_angular_ = m->angular.z; });

        pub_status_ = create_publisher<AebStatus>("/safety/aeb_status", 10);
        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/control/aeb_cmd", 10);
        pub_obstacles_ = create_publisher<visualization_msgs::msg::MarkerArray>("/safety/aeb/debug_obstacles", 10);
        pub_corridor_ = create_publisher<visualization_msgs::msg::MarkerArray>("/safety/aeb/debug_corridor", 10);

        auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]()
                                   { tick(); });
    }

private:
    double seconds_clear() const
    {
        return clear_count_ / std::max(1.0, rate_hz_);
    }

    std::string reason_string() const
    {
        switch (state_)
        {
        case AebState::INACTIVE:
            return "inactive";
        case AebState::MONITORING:
            return "monitoring, clear";
        case AebState::WARNING:
            return "warning: ttc or distance below soft threshold";
        case AebState::BRAKING:
            return "braking: ttc or distance below hard threshold";
        case AebState::STOP_HOLD:
            return "stopped, holding";
        case AebState::FAULT:
            return "fault: stale or missing input";
        }
        return "";
    }

    // Transition table. persist_count_ gates ENTRY into
    // WARNING/BRAKING (do not trigger from one noisy frame); clear_count_
    // gates EXIT back toward MONITORING (do not release the instant a cluster
    // flickers out for a frame - "use hold time and release hysteresis").

    void tick()
    {
    }

private:
    Eigen::Isometry3d T_base_cam_;
    av::proj::Pinhole pin_;
    av::safety::ObstacleFilterParams fp_;
    av::safety::FootprintParams foot_;
    double pred_t_ = 3.0, pred_dt_ = 0.1, mpc_path_timeout_ = 0.3;
    double t_response_ = 0.3, a_decel_ = 6.0, d_offset_ = 1.0;
    double warning_ttc_ = 2.5, brake_ttc_ = 1.5, collision_hold_ = 1.0,
           clear_hysteresis_ = 0.5, stop_hold_speed_ = 0.1;
    int persistence_ticks_ = 3;
    double rate_hz_ = 20.0, odom_stale_ = 0.3, depth_stale_ = 0.3;

    AebState state_ = AebState::INACTIVE;
    int persist_count_ = 0, clear_count_ = 0;
    double last_nominal_angular_ = 0.0;

    sensor_msgs::msg::Image::ConstSharedPtr depth_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    nav_msgs::msg::Path::ConstSharedPtr mpc_path_;
    rclcpp::Time depth_t_{0, 0, RCL_ROS_TIME}, odom_t_{0, 0, RCL_ROS_TIME}, mpc_path_t_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_mpc_path_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nominal_cmd_;

    rclcpp::Publisher<AebStatus>::SharedPtr pub_status_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_, pub_corridor_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AebNode>());
    rclcpp::shutdown();
    return 0;
}