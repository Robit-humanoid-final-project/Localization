#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <humanoid_interfaces/msg/vision2_local_observ.hpp>
#include "vision_observation.hpp"


class VisionSubscriber : public rclcpp::Node
{
public:
    using VisionMsg = humanoid_interfaces::msg::Vision2LocalObserv;

    explicit VisionSubscriber()
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

    bool getLatestObservation(VisionObservation& observation) const
    {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        if (!has_observation_)
            return false;

        observation = latest_observation_;
        return true;
    }

    // Return each received frame once for state-estimation updates.
    bool takeNewObservation(VisionObservation& observation)
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

    

private:
    void VisionCallback(const VisionMsg& msg)
    {
        VisionObservation observation;

        // 메시지의 header를 내부 구조체에 복사   
        observation.stamp_sec = msg.header.stamp.sec;
        observation.stamp_nanosec = msg.header.stamp.nanosec;
        observation.frame_id = msg.header.frame_id;

        // Vision 상태 값
        observation.status = static_cast<VisionStatus>(msg.status);

        // 이번 프레임의 랜드마크 목록을 새로 채움
        observation.detections.clear();

        for (const auto& item : msg.detections)
        {
            LandmarkObservation landmark;
            landmark.type = item.type;
            landmark.detection_id = item.detection_id;
            landmark.confidence = item.confidence;
            landmark.x = item.x;
            landmark.y = item.y;

            observation.detections.push_back(landmark);
        }

        // 공은 최대 한 개라고 정한 구조
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

        // 장애물은 여러 개일 수 있으므로 모두 복사
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

        // 여기서 observation을 UI 쪽 또는 기존 처리 함수로 전달
        {
            std::lock_guard<std::mutex> lock(observation_mutex_);
            latest_observation_ = observation;
            has_observation_ = true;
            has_new_observation_ = true;
        }

        RCLCPP_INFO(
            get_logger(),
            "Vision 수신: landmarks=%zu, ball=%zu, obstacles=%zu",
            observation.detections.size(),
            observation.ball.size(),
            observation.obstacles.size());
    }

    
    rclcpp::Subscription<VisionMsg>::SharedPtr vision_subscription_;

    mutable std::mutex observation_mutex_;

    VisionObservation latest_observation_;
    bool has_observation_ = false;
    bool has_new_observation_ = false;
};
