#include "odometry.hpp"

#include <cmath>

// 생성자
Odometry::Odometry(const RobotPose& initial_pose)
    : pose_(initial_pose)
{
    pose_.yaw = normalizeYaw(pose_.yaw);
}

//Odometry 이동량 적용 
void Odometry::update(const OdometryMove& move)
{
    //현재 로봇 방향
    const double cos_yaw = std::cos(pose_.yaw);
    const double sin_yaw = std::sin(pose_.yaw);

    //로봇 좌표계 이동량 -> 월드 좌표계로 변환 
    pose_.x += move.dx * cos_yaw - move.dy * sin_yaw;
    pose_.y += move.dx * sin_yaw + move.dy * cos_yaw;

    // 방향 갱신 
    pose_.yaw = normalizeYaw(pose_.yaw + move.dyaw);
}

// 현재 위치 반환
const RobotPose& Odometry::getPose() const
{
    return pose_;
}

// 위치 초기화
void Odometry::reset(const RobotPose& pose)
{
    pose_ = pose;
    pose_.yaw = normalizeYaw(pose_.yaw);
}

// Yaw 정규화
double Odometry::normalizeYaw(double yaw) const
{
    constexpr double PI = 3.14159265358979323846;

    while (yaw >= PI)
        yaw -= 2.0 * PI;

    while (yaw < -PI)
        yaw += 2.0 * PI;

    return yaw;
}