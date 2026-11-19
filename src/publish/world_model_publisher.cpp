#include <vector>
#include <utility>

#include "publish/world_model_publisher.hpp"

WorldModelPublisher::WorldModelPublisher()
    : Node("world_model_publisher")
{
    publisher_ = this->create_publisher<humanoid_interfaces::msg::WorldModel>(
        "/world_model",
        10
    );
}

void WorldModelPublisher::publishWorldModel(double robot_x, double robot_y, double robot_yaw, bool ball_detected, double ball_x, double ball_y, const std::vector<std::pair<double, double>>& obstacles)
{
    humanoid_interfaces::msg::WorldModel msg;

    // 로봇 월드 좌표
    msg.robot_pose.x = robot_x;
    msg.robot_pose.y = robot_y;
    msg.robot_pose.theta = robot_yaw;

    // 위치 추정 수렴 여부: 추후 구현
    msg.is_localized = false;

    // 공 월드 좌표
    msg.ball_detected = ball_detected;

    if (ball_detected)
    {
        msg.ball_pose.x = ball_x;
        msg.ball_pose.y = ball_y;
    }

    // 장애물 월드 좌표
    for (const auto& obstacle : obstacles)
    {
        geometry_msgs::msg::Point point;

        point.x = obstacle.first;
        point.y = obstacle.second;

        msg.obstacles.push_back(point);
    }

    publisher_->publish(msg);
}