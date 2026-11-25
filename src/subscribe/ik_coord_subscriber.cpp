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

bool IkCoordSubscriber::takeIkUpdate(std::int64_t& x, std::int64_t& y)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_update_)
    {
        return false;
    }

    x = ikcoordinated_x;
    y = ikcoordinated_y;
    ikcoordinated_x = 0;
    ikcoordinated_y = 0;
    has_update_ = false;
    return true;
}

void IkCoordSubscriber::IkCoordCallback(const IkCoordMsg& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ikcoordinated_x += msg.x;
    ikcoordinated_y += msg.y;
    has_update_ = true;
}
