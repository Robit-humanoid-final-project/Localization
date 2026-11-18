#include "subscribe/ik_coord_subscriber.hpp"

#include <mutex>

IkCoordSubscriber::IkCoordSubscriber()
    : Node("ik_coord_subscriber")
{
    subscription_ = create_subscription<IkCoordMsg>(
        "/ikcoordinate",
        rclcpp::QoS(10).reliable(),
        [this](const IkCoordMsg::SharedPtr msg)
        {
            IkCoordCallback(*msg);
        });
}

bool IkCoordSubscriber::takeUpdate(std::int64_t& x, std::int64_t& y)
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

void IkCoordSubscriber::IkCoordCallback(const IkCoordMsg& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    pending_x_ += msg.x;
    pending_y_ += msg.y;
    has_update_ = true;
}
