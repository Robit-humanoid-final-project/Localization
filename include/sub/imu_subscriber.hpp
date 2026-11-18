#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <humanoid_interfaces/msg/imu_msg.hpp>
#include "robot_state.hpp"

class ImuSubscriber : public rclcpp::Node
{
public:
    using ImuMsg = humanoid_interfaces::msg::ImuMsg;

    explicit ImuSubscriber()
        : Node("imu_subscriber")
    {
        imu_subscription_ = create_subscription<ImuMsg>(
            "/Imu",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            [this](const ImuMsg::SharedPtr msg)
            {
                ImuCallback(*msg);
            });
    }

    bool takeImuUpdate(double& yaw_deg, double& dyaw_rad)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        if(!has_new_imu_)
        {
            return false;
        }

        yaw_deg = latest_yaw_deg_;
        dyaw_rad = pending_dyaw_rad_;

        pending_dyaw_rad_ = 0.0;
        has_new_imu_ = false;
        return true;
    }

private: 
    void ImuCallback(const ImuMsg& msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        const double current_yaw_deg = msg.yaw;
        double delta_deg = 0.0;

        if(has_previous_yaw_)
        {
            delta_deg = current_yaw_deg - previous_yaw_deg_;

            // yaw가 +180°/-180° 경계를 통과할 때 차이를 보정
            while (delta_deg > 180.0)
            {
                delta_deg -= 360.0;
            }
            while (delta_deg < -180.0)
            {
                delta_deg += 360.0;
            }

            constexpr double pi = 3.14159265358979323846;
            pending_dyaw_rad_ += delta_deg * pi / 180.0;
        }

        previous_yaw_deg_ = current_yaw_deg;
        latest_yaw_deg_ = current_yaw_deg;
        has_previous_yaw_ = true;
        has_new_imu_ = true;
    }

    rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscription_;
    mutable std::mutex imu_mutex_;

    double previous_yaw_deg_ = 0.0;
    double latest_yaw_deg_ = 0.0;
    double pending_dyaw_rad_ = 0.0;
    bool has_previous_yaw_ = false;
    bool has_new_imu_ = false;
};

