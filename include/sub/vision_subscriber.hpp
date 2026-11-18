#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <humanoid_interfaces/msg/vision2_local_observ.hpp>

#include "vision_observation.hpp"

class VisionSubscriber : public rclcpp::Node
{
public:
    using VisionMsg = humanoid_interfaces::msg::Vision2LocalObserv;

    VisionSubscriber();
    bool getLatestObservation(VisionObservation& observation) const;
    bool takeNewObservation(VisionObservation& observation);

private:
    void VisionCallback(const VisionMsg& msg);

    rclcpp::Subscription<VisionMsg>::SharedPtr vision_subscription_;
    mutable std::mutex observation_mutex_;
    VisionObservation latest_observation_;
    bool has_observation_ = false;
    bool has_new_observation_ = false;
};
