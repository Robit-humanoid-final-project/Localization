#pragma once

#include <cstdint>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <humanoid_interfaces/msg/ik_coord_msg.hpp>

class IkCoordSubscriber : public rclcpp::Node
{
public:
    using IkCoordMsg = humanoid_interfaces::msg::IkCoordMsg;

    explicit IkCoordSubscriber()
        : Node("ik_coord_subscriber")
    {
        subscription_ = create_subscription<IkCoordMsg>(
            "/ikcoordinate",
            rclcpp::QoS(10).reliable(),
            [this](const IkCoordMsg::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                pending_x_ += msg->x;
                pending_y_ += msg->y;
                has_update_ = true;
            });
    }

    bool takeUpdate(std::int64_t& x, std::int64_t& y)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_update_)
        {
            return false;
        }

        x = pending_x_;
        y = pending_y_;
        pending_x_ = 0;
        pending_y_ = 0;
        has_update_ = false;
        
        return true;
    }

private:
    rclcpp::Subscription<IkCoordMsg>::SharedPtr subscription_;
    std::mutex mutex_;
    std::int64_t pending_x_ = 0;
    std::int64_t pending_y_ = 0;
    bool has_update_ = false;
};
