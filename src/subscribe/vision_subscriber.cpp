#include "subscribe/vision_subscriber.hpp"

#include <mutex>

VisionSubscriber::VisionSubscriber()
    : Node("mcl_vision_subscriber")
{
    vision_subscription_ = create_subscription<VisionMsg>(
        "/vision_packages",
        rclcpp::QoS(10),
        [this](const VisionMsg::SharedPtr msg)
        {
            VisionCallback(*msg);
        });
}

bool VisionSubscriber::getLatestObservation(VisionObservation& observation) const
{
    std::lock_guard<std::mutex> lock(observation_mutex_);
    if (!has_observation_)
    {
        return false;
    }

    observation = latest_observation_;
    return true;
}

bool VisionSubscriber::takeNewObservation(VisionObservation& observation)
{
    std::lock_guard<std::mutex> lock(observation_mutex_);
    if (!has_new_observation_)
    {
        return false;
    }

    observation = latest_observation_;
    has_new_observation_ = false;
    return true;
}

void VisionSubscriber::VisionCallback(const VisionMsg& msg)
{
    VisionObservation observation;
    observation.stamp_sec = msg.header.stamp.sec;
    observation.stamp_nanosec = msg.header.stamp.nanosec;
    observation.frame_id = msg.header.frame_id;
    observation.status = static_cast<VisionStatus>(msg.status);

    observation.detections.clear();
    for (const auto& item : msg.detections)
    {
        LandmarkObservation landmark;
        landmark.type = item.type; // detection[].type 토픽 
        landmark.detection_id = item.detection_id; // detection[].detection_id 토픽
        landmark.confidence = item.confidence; // detection[].confidence 토픽
        landmark.x = item.x; // detection[].x 토픽
        landmark.y = item.y; // detection[].y 토픽
        observation.detections.push_back(landmark);
    }

    observation.ball.clear();
    const float ball_x = msg.ball[0];
    const float ball_y = msg.ball[1];
    const float ball_confidence = msg.ball[2];
    if (ball_x != 0.0F || ball_y != 0.0F || ball_confidence != 0.0F)
    {
        BallObservation ball;
        ball.x = ball_x;
        ball.y = ball_y;
        ball.confidence = ball_confidence;
        observation.ball.push_back(ball);
    }

    observation.obstacles.clear();
    for (const auto& item : msg.obstacles)
    {
        ObstacleObservation obstacle;
        obstacle.detection_id = item.detection_id;
        obstacle.confidence = item.confidence;
        obstacle.x = item.x;
        obstacle.y = item.y;
        observation.obstacles.push_back(obstacle);
    }

    {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        latest_observation_ = observation;
        has_observation_ = true;
        has_new_observation_ = true;
    }
}
