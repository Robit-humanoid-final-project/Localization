#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include "robot_state.hpp"

class Odometry
{
public:

    //초기 위치 설정 
    explicit Odometry(const RobotPose& initial_pose = RobotPose{});

    //Odometry 이동량 -> 현재 위치 갱신
    void update(const OdometryMove& move);

    //현재 위치 반환 
    const RobotPose& getPose() const;

    //위치 초기화
    void reset(const RobotPose& pose = RobotPose{});

private:
    RobotPose pose_;

    //Yaw 값 정규화(-PI ~ PI)
    double normalizeYaw(double yaw) const;

};

#endif