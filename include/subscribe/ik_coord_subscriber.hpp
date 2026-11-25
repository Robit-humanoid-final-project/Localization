#pragma once

#include <cstdint>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <humanoid_interfaces/msg/ik_coord_msg.hpp>

class IkCoordSubscriber : public rclcpp::Node
{
public:
    using IkCoordMsg = humanoid_interfaces::msg::IkCoordMsg;

    IkCoordSubscriber();
    bool takeIkUpdate(std::int64_t& x, std::int64_t& y);

private:
    void IkCoordCallback(const IkCoordMsg& msg);

    rclcpp::Subscription<IkCoordMsg>::SharedPtr subscription_;
    std::mutex mutex_;
    std::int64_t ikcoordinated_x = 0;
    std::int64_t ikcoordinated_y = 0;
    bool has_update_ = false;
};
