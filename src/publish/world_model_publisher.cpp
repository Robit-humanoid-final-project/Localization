#include <vector>
#include <utility>

#include "publish/world_model_publisher.hpp"

WorldModelPublisher::WorldModelPublisher()
    : Node("world_model_publisher"), 
      last_object_publish_time_(this->now())
{
    publisher_ = this->create_publisher<humanoid_interfaces::msg::WorldModel>(
        "/world_model",
        10
    );
}

void WorldModelPublisher::publishWorldModel(double robot_x, double robot_y, double robot_yaw, bool ball_detected, double ball_x, double ball_y, const std::vector<std::pair<double, double>>& obstacles)
{
    // 현재 시간
    rclcpp::Time current_time = this->now();

    if ((current_time - last_object_publish_time_).seconds() >= 3.0)
    {
        ball_detected_ = ball_detected;
        ball_x_ = ball_x;
        ball_y_ = ball_y;

        obstacles_.clear();

        for (const auto& obstacle : obstacles)
        {
            geometry_msgs::msg::Point point;
            point.x = obstacle.first;
            point.y = obstacle.second;

            obstacles_.push_back(point);
        }

        last_object_publish_time_ = current_time;
    }

    humanoid_interfaces::msg::WorldModel msg;

    // 로봇 월드 좌표
    msg.robot_pose.x = robot_x;
    msg.robot_pose.y = robot_y;
    msg.robot_pose.theta = robot_yaw;

    // 위치 추정 수렴 여부: 아직 구현 못함
    msg.is_localized = false;


    // 공 월드 좌표
    msg.ball_detected = ball_detected_;

    if (ball_detected_)
    {
        msg.ball_pose.x = ball_x_;
        msg.ball_pose.y = ball_y_;
    }

    // 저장된 장애물 정보
    msg.obstacles = obstacles_;

    publisher_->publish(msg);
}