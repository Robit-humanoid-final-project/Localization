#ifndef WORLD_MODEL_PUBLISHER_HPP
#define WORLD_MODEL_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <humanoid_interfaces/msg/world_model.hpp>

class WorldModelPublisher : public rclcpp::Node
{
public:
    WorldModelPublisher();

    void publishWorldModel(
        double robot_x,
        double robot_y,
        double robot_yaw,
        bool ball_detected,
        double ball_x,
        double ball_y,
        const std::vector<std::pair<double, double>>& obstacles
    );

private:
    rclcpp::Publisher<humanoid_interfaces::msg::WorldModel>::SharedPtr publisher_;
    rclcpp::Time last_object_publish_time_;

    bool ball_detected_ = false;

    double ball_x_ = 0.0;
    double ball_y_ = 0.0;

    std::vector<geometry_msgs::msg::Point> obstacles_;
};

#endif