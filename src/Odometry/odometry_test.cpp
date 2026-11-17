#include "odometry.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>

int main()
{
    constexpr double PI = 3.14159265358979323846;

    Odometry odometry;

    // 테스트 1: 초기 방향 0도에서 전방 1m 이동
    std::cout << "===== Test 1 =====\n";

    odometry.update({1.0, 0.0, 0.0});

    RobotPose pose = odometry.getPose();

    std::cout << "x: " << pose.x
              << ", y: " << pose.y
              << ", yaw: " << pose.yaw << "\n";

    // 테스트 2: 방향을 90도로 변경한 후 전방 1m 이동
    std::cout << "\n===== Test 2 =====\n";

    odometry.reset({0.0, 0.0, PI / 2.0});
    odometry.update({1.0, 0.0, 0.0});

    pose = odometry.getPose();

    std::cout << "x: " << pose.x
              << ", y: " << pose.y
              << ", yaw: " << pose.yaw << "\n";

    // 테스트 3: 방향이 90도일 때 좌측으로 1m 이동
    std::cout << "\n===== Test 3 =====\n";

    odometry.reset({0.0, 0.0, PI / 2.0});
    odometry.update({0.0, 1.0, 0.0});

    pose = odometry.getPose();

    std::cout << "x: " << pose.x
              << ", y: " << pose.y
              << ", yaw: " << pose.yaw << "\n";

    // 테스트 4: Yaw 범위 정규화
    std::cout << "\n===== Test 4 =====\n";

    odometry.reset({0.0, 0.0, 3.0 * PI});

    pose = odometry.getPose();

    std::cout << "yaw: " << pose.yaw << "\n";

    return 0;
}
