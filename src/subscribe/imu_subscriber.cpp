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

    // 최초 IMU 수신 시 월드 좌표계 기준 방향 설정
    if (!has_initial_yaw_)
    {
        initial_yaw_deg_ = current_yaw_deg;
        previous_yaw_deg_ = current_yaw_deg;
        has_initial_yaw_ = true;
        has_previous_yaw_ = true;
    }

    // IMU 원본 Yaw -> 월드 좌표계 Yaw
    double world_yaw_deg = -(current_yaw_deg - initial_yaw_deg_);


    // -180 ~ +180도 범위로 정규화
    while (world_yaw_deg > 180.0)
    {
        world_yaw_deg -= 360.0;
    }

    while (world_yaw_deg < -180.0)
    {
        world_yaw_deg += 360.0;
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
    
    RCLCPP_INFO(
        get_logger(),
        "current=%.2f, previous=%.2f, delta=%.2f, has_previous=%d",
        current_yaw_deg,
        previous_yaw_deg_,
        delta_deg,
        has_previous_yaw_
    );

    previous_yaw_deg_ = current_yaw_deg;
    latest_yaw_deg_ = world_yaw_deg;
    has_previous_yaw_ = true;
    has_new_imu_ = true;

}
