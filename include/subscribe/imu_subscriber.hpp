#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <humanoid_interfaces/msg/imu_msg.hpp>

class ImuSubscriber : public rclcpp::Node
{
public:
    using ImuMsg = humanoid_interfaces::msg::ImuMsg;

    ImuSubscriber();
    bool takeImuUpdate(double& yaw_deg, double& dyaw_rad);

private:
    void ImuCallback(const ImuMsg& msg);

    rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscription_;
    std::mutex imu_mutex_;

    double initial_yaw_deg_ = 0.0;
    bool has_initial_yaw_ = false;
    double previous_yaw_deg_ = 0.0;
    double latest_yaw_deg_ = 0.0;
    double pending_dyaw_rad_ = 0.0;
    bool has_previous_yaw_ = false;
    bool has_new_imu_ = false;
};
