#include "subscribe/imu_subscriber.hpp"

#include <cmath>
#include <mutex>

ImuSubscriber::ImuSubscriber()
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

bool ImuSubscriber::takeImuUpdate(double& yaw_deg, double& dyaw_rad)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!has_new_imu_)
    {
        return false;
    }

    yaw_deg = latest_yaw_deg_;
    dyaw_rad = pending_dyaw_rad_;
    pending_dyaw_rad_ = 0.0;
    has_new_imu_ = false;
    return true;
}

void ImuSubscriber::ImuCallback(const ImuMsg& msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);

    const double current_yaw_deg = msg.yaw;
    if (!has_previous_yaw_)
    {
        previous_yaw_deg_ = current_yaw_deg;
        has_previous_yaw_ = true;
    }

    // IMU 회전량 계산
    double delta_deg = current_yaw_deg - previous_yaw_deg_;
    while (delta_deg > 180.0)
    {
        delta_deg -= 360.0;
    }
    while (delta_deg < -180.0)
    {
        delta_deg += 360.0;
    }

    constexpr double pi = 3.14159265358979323846;
    pending_dyaw_rad_ += -delta_deg * pi / 180.0;

    previous_yaw_deg_ = current_yaw_deg;
    latest_yaw_deg_ = current_yaw_deg;
    has_previous_yaw_ = true;
    has_new_imu_ = true;

}
