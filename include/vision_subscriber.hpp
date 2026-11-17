#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// 실제 Vision 메시지 패키지와 타입 이름으로 바꿔야 함
#include <vision_interfaces/msg/vision_packages.hpp>

#include "vision_observation.hpp"

class VisionSubscriber : public rclcpp::Node
{
public:
    using VisionMsg = vision_interfaces::msg::VisionPackages;

    explicit VisionSubscriber()
        : Node("mcl_vision_subscriber")
    {
        subscription_ = create_subscription<VisionMsg>(
            "/vision_packages",
            rclcpp::QoS(10),
            [this](const VisionMsg::SharedPtr msg)
            {
                onVisionMessage(*msg);
            });
    }

private:
    void onVisionMessage(const VisionMsg& msg)
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

        for (const auto& item : msg.detection)
        {
            LandmarkObservation landmark;
            landmark.type = static_cast<LandmarkType>(item.type);
            landmark.detection_id = item.detection_id;
            landmark.confidence = item.confidence;
            landmark.x = item.x;
            landmark.y = item.y;

            observation.detections.push_back(landmark);
        }

        // 공은 최대 한 개라고 정한 구조
        observation.ball.clear();

        if (!msg.ball.empty())
        {
            BallObservation ball;
            ball.x = msg.ball[0].x;
            ball.y = msg.ball[0].y;
            ball.confidence = msg.ball[0].confidence;

            observation.ball.push_back(ball);
        }

        // 장애물은 여러 개일 수 있으므로 모두 복사
        observation.obstacles.clear();

        for (const auto& item : msg.obstacle)
        {
            ObstacleObservation obstacle;
            obstacle.detection_id = item.detection_id;
            obstacle.confidence = item.confidence;
            obstacle.x = item.x;
            obstacle.y = item.y;

            observation.obstacles.push_back(obstacle);
        }

        // 여기서 observation을 UI 쪽 또는 기존 처리 함수로 전달
        latest_observation_ = observation;

        RCLCPP_INFO(
            get_logger(),
            "Vision 수신: landmarks=%zu, ball=%zu, obstacles=%zu",
            observation.detections.size(),
            observation.ball.size(),
            observation.obstacles.size());
    }

    rclcpp::Subscription<VisionMsg>::SharedPtr subscription_;
    VisionObservation latest_observation_;
};